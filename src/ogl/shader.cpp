/*!
 * \file shader.cpp
 *
 * \author Han
 * \date 2017/04/06
 *
 *
 */
#include <fstream>

#include <ogl/shader.h>

#include <QOpenGLShader>

namespace h2o {
GLShader::GLShader() {

    CHECK(initializeOpenGLFunctions());
    own_program_ = true;
}

bool GLShader::initFromFiles(const std::string &name, const std::string &vertex_fname,
                             const std::string &fragment_fname, const std::string &geometry_fname) {
    auto file_to_string = [](const std::string &filename) -> std::string {
        if (filename.empty())
            return "";
        std::ifstream t(filename);
        return std::string((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    };

    return init(name, file_to_string(vertex_fname), file_to_string(fragment_fname), file_to_string(geometry_fname));
}

bool GLShader::init(const std::string &name, const std::shared_ptr<QOpenGLShaderProgram> &program) {
    program_ = program;
    own_program_ = false;
    vao_.create();
    CHECK(vao_.isCreated());
    name_ = name;
    if (!program_->isLinked()) {
        std::cerr << "Linker error (" << name_ << "): " << std::endl << program_->log().constData() << std::endl;
        throw std::runtime_error("Shader linking failed!");
    }
    return true;
}

bool GLShader::init(const std::string &name, const std::string &vertex_str, const std::string &fragment_str,
                    const std::string &geometry_str) {
    program_ = std::make_shared<QOpenGLShaderProgram>();
    own_program_ = true;

    std::string defines;
    for (auto def : definitions_)
        defines += std::string("#define ") + def.first + std::string(" ") + def.second + "\n";

    vao_.create();
    CHECK(vao_.isCreated());

    name_ = name;
    createShader_helper(QOpenGLShader::Vertex, name, defines, vertex_str);
    createShader_helper(QOpenGLShader::Fragment, name, defines, fragment_str);

    if (!geometry_str.empty()) {
        createShader_helper(QOpenGLShader::Geometry, name, defines, geometry_str);
    }
    program_->link();
    if (!program_->isLinked()) {
        std::cerr << "Linker error (" << name_ << "): " << std::endl << program_->log().constData() << std::endl;
        throw std::runtime_error("Shader linking failed!");
    }
    return true;
}

void GLShader::bind() {
    program_->bind();
    vao_.bind();
}

void GLShader::release() {
    program_->release();
    vao_.release();
}

GLint GLShader::attrib(const std::string &name, bool warn) const {
    GLint id = program_->attributeLocation(name.c_str());
    if (id == -1 && warn)
        std::cerr << name_ << ": warning: did not find attrib " << name << std::endl;
    return id;
}

GLint GLShader::uniform(const std::string &name, bool warn) const {
    GLint id = program_->uniformLocation(name.c_str());
    if (id == -1 && warn)
        std::cerr << name_ << ": warning: did not find uniform " << name << std::endl;
    return id;
}

void GLShader::uploadAttrib(const std::string &name, size_t size, int dim, uint32_t component_size, GLuint gltype,
                            bool integral, const void *data, int version) {
    int id_attrib = 0;
    if (name != "indices") {
        id_attrib = attrib(name);
        if (id_attrib < 0)
            return;
    }
    QOpenGLBuffer *qt_buffer;
    auto it = buffer_object_.find(name);
    if (it != buffer_object_.end()) {
        Buffer &buffer = it->second;
        qt_buffer = it->second.qt_buffer;
        buffer.version = version;
        buffer.size = size;
        buffer.component_size = component_size;
    } else {
        if (name == "indices") {
            qt_buffer = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        } else {
            qt_buffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        }
        qt_buffer->create();
        Buffer buffer;
        buffer.qt_buffer = qt_buffer;
        buffer.gltype = gltype;
        buffer.dim = dim;
        buffer.component_size = component_size;
        buffer.size = size;
        buffer.version = version;
        buffer_object_[name] = buffer;
    }
    size_t total_size = size * (size_t)component_size;
    qt_buffer->bind();
    qt_buffer->allocate(data, total_size);
    if (name != "indices") {
        if (size == 0) {
            glDisableVertexAttribArray(id_attrib);
        } else {
            glEnableVertexAttribArray(id_attrib);
            glVertexAttribPointer(id_attrib, dim, gltype, integral, 0, 0);
        }
    }
}

void GLShader::downloadAttrib(const std::string &name, size_t size, int /* dim */, uint32_t compSize,
                              GLuint /* glType */, void *data) {
    auto it = buffer_object_.find(name);
    if (it == buffer_object_.end())
        throw std::runtime_error("downloadAttrib(" + name_ + ", " + name + ") : buffer not found!");

    const Buffer &buf = it->second;
    if (buf.size != size || buf.component_size != compSize)
        throw std::runtime_error(name_ + ": downloadAttrib: size mismatch!");

    size_t total_size = size * (size_t)compSize;
    buf.qt_buffer->bind();
    buf.qt_buffer->read(0, data, total_size);
}

bool GLShader::createShader_helper(QOpenGLShader::ShaderType type, const std::string &name, const std::string &defines,
                                   std::string shader_string) {
    if (shader_string.empty())
        return false;

    if (!defines.empty()) {
        if (shader_string.length() > 8 && shader_string.substr(0, 8) == "#version") {
            std::istringstream iss(shader_string);
            std::ostringstream oss;
            std::string line;
            std::getline(iss, line);
            oss << line << std::endl;
            oss << defines;
            while (std::getline(iss, line))
                oss << line << std::endl;
            shader_string = oss.str();
        } else {
            shader_string = defines + shader_string;
        }
    }

    return program_->addShaderFromSourceCode((QOpenGLShader::ShaderType)type, shader_string.c_str());
}

void GLShader::shareAttrib(const GLShader &otherShader, const std::string &name, const std::string &_as) {
    std::string as = _as.length() == 0 ? name : _as;
    auto it = otherShader.buffer_object_.find(name);
    if (it == otherShader.buffer_object_.end())
        throw std::runtime_error("shareAttribute(" + otherShader.name_ + ", " + name + "): attribute not found!");
    const Buffer &buffer = it->second;

    if (name != "indices") {
        int id_attrib = attrib(as);
        if (id_attrib < 0)
            return;
        buffer.qt_buffer->bind();
        glEnableVertexAttribArray(id_attrib);
        glVertexAttribPointer(id_attrib, buffer.dim, buffer.gltype, buffer.component_size == 1 ? GL_TRUE : GL_FALSE, 0,
                              0);
    } else {
        buffer.qt_buffer->bind();
    }
}

void GLShader::invalidateAttribs() {
    for (auto &buffer : buffer_object_)
        buffer.second.version = -1;
}

void GLShader::freeAttrib(const std::string &name) {
    auto it = buffer_object_.find(name);
    if (it != buffer_object_.end()) {
        it->second.qt_buffer->destroy();
        buffer_object_.erase(it);
    }
}

void GLShader::drawIndexed(int type, uint32_t offset_, uint32_t count_) {
    if (count_ == 0)
        return;
    size_t offset = offset_;
    size_t count = count_;

    switch (type) {
    case GL_TRIANGLES:
        offset *= 3;
        count *= 3;
        break;
    case GL_LINES:
        offset *= 2;
        count *= 2;
        break;
    }

    glDrawElements(type, (GLsizei)count, GL_UNSIGNED_INT, (const void *)(offset * sizeof(uint32_t)));
}

void GLShader::drawArray(int type, uint32_t offset, uint32_t count) {
    if (count == 0)
        return;

    glDrawArrays(type, offset, count);
}

void GLShader::free() {
    for (auto &buf : buffer_object_) {
        buf.second.qt_buffer->release();
        buf.second.qt_buffer->destroy();
    }
    buffer_object_.clear();

    if (vao_.isCreated()) {
        vao_.release();
        vao_.destroy();
    }

    if (program_ && program_->isLinked() && own_program_) {
        program_->release();
        program_->removeAllShaders();
    }
}
} // end h2o
