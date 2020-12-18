/*!
 * \file shader.h
 *
 * \author Han
 * \date 2017/04/06
 *
 *
 */
#pragma once
#include <base/common.h>

#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <half.hpp>

#define CHECK_OPENGL_ERROR()                                                                                           \
    {                                                                                                                  \
        auto error_code = glGetError();                                                                                \
        CHECK(error_code == GL_NO_ERROR) << error_code;                                                                \
    }

namespace h2o {
namespace detail {
template <typename T> struct type_traits;
template <> struct type_traits<uint32_t> {
    enum { type = GL_UNSIGNED_INT, integral = 1 };
};
template <> struct type_traits<int32_t> {
    enum { type = GL_INT, integral = 1 };
};
template <> struct type_traits<uint16_t> {
    enum { type = GL_UNSIGNED_SHORT, integral = 1 };
};
template <> struct type_traits<int16_t> {
    enum { type = GL_SHORT, integral = 1 };
};
template <> struct type_traits<uint8_t> {
    enum { type = GL_UNSIGNED_BYTE, integral = 1 };
};
template <> struct type_traits<int8_t> {
    enum { type = GL_BYTE, integral = 1 };
};
template <> struct type_traits<double> {
    enum { type = GL_DOUBLE, integral = 0 };
};
template <> struct type_traits<float> {
    enum { type = GL_FLOAT, integral = 0 };
};
template <> struct type_traits<half_float::half> {
    enum { type = GL_HALF_FLOAT, integral = 0 };
};
template <typename T> struct serialization_helper;
} // namespace detail
class GLShader : public QOpenGLFunctions_3_3_Core {
public:
    GLShader();
    ~GLShader() { free(); }

    /**
     * \brief Initialize the shader using the specified source strings.
     *
     * \param name
     *     The name this shader will be registered as.
     *
     * \param vertex_str
     *     The source of the vertex shader as a string.
     *
     * \param fragment_str
     *     The source of the fragment shader as a string.
     *
     * \param geometry_str
     *     The source of the geometry shader as a string.  The default value is
     *     the empty string, which indicates no geometry shader will be used.
     */
    bool init(const std::string &name, const std::string &vertex_str, const std::string &fragment_str,
              const std::string &geometry_str = "");

    /**
     * \brief Initialize the shader using the specified files on disk.
     *
     * \param name
     *     The name this shader will be registered as.
     *
     * \param vertex_fname
     *     The path to the file containing the source of the fragment shader.
     *
     * \param fragment_fname
     *     The path to the file containing the source of the vertex shader.
     *
     * \param geometry_fname
     *     The path to the file containing the source of the geometry shader.
     *     The default value is the empty string, which indicates no geometry
     *     shader will be used.
     */
    bool initFromFiles(const std::string &name, const std::string &vertex_fname, const std::string &fragment_fname,
                       const std::string &geometry_fname = "");

    bool init(const std::string &name, const std::shared_ptr<QOpenGLShaderProgram> &program);

    /// Return the name of the shader
    const std::string &name() const { return name_; }

    /// Set a preprocessor definition
    void define(const std::string &key, const std::string &value) { definitions_[key] = value; }

    /// Select this shader for subsequent draw calls
    void bind();

    void release();

    /// Release underlying OpenGL objects
    void free();

    /// Return the handle of a named shader attribute (-1 if it does not exist)
    GLint attrib(const std::string &name, bool warn = true) const;

    /// Return the handle of a uniform attribute (-1 if it does not exist)
    GLint uniform(const std::string &name, bool warn = true) const;

    /// Upload an Eigen matrix as a vertex buffer object (refreshing it as needed)
    template <typename Matrix> void uploadAttrib(const std::string &name, const Matrix &M, int version = -1) {
        uint32_t compSize = sizeof(typename Matrix::Scalar);
        GLuint glType = (GLuint)detail::type_traits<typename Matrix::Scalar>::type;
        bool integral = (bool)detail::type_traits<typename Matrix::Scalar>::integral;

        uploadAttrib(name, (uint32_t)M.size(), (int)M.rows(), compSize, glType, integral, M.data(), version);
    }

    /// Download a vertex buffer object into an Eigen matrix
    template <typename Matrix> void downloadAttrib(const std::string &name, Matrix &M) {
        uint32_t compSize = sizeof(typename Matrix::Scalar);
        GLuint glType = (GLuint)detail::type_traits<typename Matrix::Scalar>::type;

        auto it = buffer_object_.find(name);
        if (it == buffer_object_.end())
            throw std::runtime_error("downloadAttrib(" + name_ + ", " + name + ") : buffer not found!");

        const Buffer &buf = it->second;
        M.resize(buf.dim, buf.size / buf.dim);

        downloadAttrib(name, M.size(), M.rows(), compSize, glType, M.data());
    }

    /// Upload an index buffer
    template <typename Matrix> void uploadIndices(const Matrix &M, int version = -1) {
        uploadAttrib("indices", M, version);
    }

    /// Invalidate the version numbers associated with attribute data
    void invalidateAttribs();

    /// Completely free an existing attribute buffer
    void freeAttrib(const std::string &name);

    /// Check if an attribute was registered a given name
    bool hasAttrib(const std::string &name) const {
        auto it = buffer_object_.find(name);
        if (it == buffer_object_.end())
            return false;
        return true;
    }

    /// Create a symbolic link to an attribute of another GLShader. This avoids duplicating unnecessary data
    void shareAttrib(const GLShader &otherShader, const std::string &name, const std::string &as = "");

    /// Return the version number of a given attribute
    int attribVersion(const std::string &name) const {
        auto it = buffer_object_.find(name);
        if (it == buffer_object_.end())
            return -1;
        return it->second.version;
    }

    /// Reset the version number of a given attribute
    void resetAttribVersion(const std::string &name) {
        auto it = buffer_object_.find(name);
        if (it != buffer_object_.end())
            it->second.version = -1;
    }

    /// Draw a sequence of primitives
    void drawArray(int type, uint32_t offset, uint32_t count);

    /// Draw a sequence of primitives using a previously uploaded index buffer
    void drawIndexed(int type, uint32_t offset, uint32_t count);

    /// Initialize a uniform parameter with a 4x4 matrix (float)
    template <typename T>
    void setUniform(const std::string &name, const Eigen::Matrix<T, 4, 4> &mat, bool warn = true) {
        program_->setUniformValue(name.c_str(), QMatrix4x4(mat.template cast<float>().data(), 4, 4));
    }

    /// Initialize a uniform parameter with an integer value
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 1, int>::type = 0>
    void setUniform(const std::string &name, T value, bool warn = true) {
        program_->setUniformValue(name.c_str(), (int)value);
    }

    /// Initialize a uniform parameter with a floating point value
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 0, int>::type = 0>
    void setUniform(const std::string &name, T value, bool warn = true) {
        program_->setUniformValue(name.c_str(), (float)value);
    }

    /// Initialize a uniform parameter with a 2D vector (int)
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 1, int>::type = 0>
    void setUniform(const std::string &name, const Eigen::Matrix<T, 2, 1> &v, bool warn = true) {
        glUniform2i(uniform(name, warn), (int)v.x(), (int)v.y());
    }

    /// Initialize a uniform parameter with a 2D vector (float)
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 0, int>::type = 0>
    void setUniform(const std::string &name, const Eigen::Matrix<T, 2, 1> &v, bool warn = true) {
        program_->setUniformValue(name.c_str(), (float)v.x(), (float)v.y());
    }

    /// Initialize a uniform parameter with a 3D vector (int)
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 1, int>::type = 0>
    void setUniform(const std::string &name, const Eigen::Matrix<T, 3, 1> &v, bool warn = true) {
        glUniform3i(uniform(name, warn), (int)v.x(), (int)v.y(), (int)v.z());
    }

    /// Initialize a uniform parameter with a 3D vector (float)
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 0, int>::type = 0>
    void setUniform(const std::string &name, const Eigen::Matrix<T, 3, 1> &v, bool warn = true) {
        program_->setUniformValue(name.c_str(), (float)v.x(), (float)v.y(), (float)v.z());
    }

    /// Initialize a uniform parameter with a 4D vector (int)
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 1, int>::type = 0>
    void setUniform(const std::string &name, const Eigen::Matrix<T, 4, 1> &v, bool warn = true) {
        glUniform4i(uniform(name, warn), (int)v.x(), (int)v.y(), (int)v.z(), (int)v.w());
    }

    /// Initialize a uniform parameter with a 4D vector (float)
    template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 0, int>::type = 0>
    void setUniform(const std::string &name, const Eigen::Matrix<T, 4, 1> &v, bool warn = true) {
        program_->setUniformValue(name.c_str(), (float)v.x(), (float)v.y(), (float)v.z(), (float)v.w());
    }

    void setUniform1fv(const std::string &name, const std::vector<float> &value, bool warn = true) {
        glUniform1fv(uniform(name, warn), value.size(), value.data());
    }
    void setUniform2fv(const std::string &name, const std::vector<float> &value, bool warn = true) {
        glUniform2fv(uniform(name, warn), value.size() / 2, value.data());
    }

    /// Return the size of all registered buffers in bytes
    size_t bufferSize() const {
        size_t size = 0;
        for (auto const &buf : buffer_object_)
            size += buf.second.size;
        return size;
    }

public:
    /* Low-level API */
    void uploadAttrib(const std::string &name, size_t size, int dim, uint32_t compSize, GLuint glType, bool integral,
                      const void *data, int version = -1);
    void downloadAttrib(const std::string &name, size_t size, int dim, uint32_t compSize, GLuint glType, void *data);
    bool createShader_helper(QOpenGLShader::ShaderType type, const std::string &name, const std::string &defines,
                             std::string shader_string);
    std::shared_ptr<QOpenGLShaderProgram> get_shader_program() const { return program_; }

protected:
    /**
     * \struct Buffer glutil.h nanogui/glutil.h
     *
     * A wrapper struct for maintaining various aspects of items being managed
     * by OpenGL.
     */
    struct Buffer {
        QOpenGLBuffer *qt_buffer;
        GLuint gltype;
        GLuint dim;
        GLuint component_size;
        GLuint size;
        int version;
    };
    std::string name_;

    std::map<std::string, Buffer> buffer_object_;
    std::map<std::string, std::string> definitions_;

    QOpenGLVertexArrayObject vao_;

    bool own_program_;
    std::shared_ptr<QOpenGLShaderProgram> program_;
};

using GLShaderPtr = std::shared_ptr<GLShader>;

} // namespace h2o
