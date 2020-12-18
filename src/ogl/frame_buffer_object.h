/*!
 * \file frame_buffer_object.h
 *
 * \author Han
 * \date 2017/05/13
 *
 * fbo 结构
 */
#pragma once
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>

#include <base/common.h>

namespace h2o {
class GLFrameBufferObject : public QOpenGLFunctions_3_3_Core {
public:
    GLFrameBufferObject(int samples = 0);
    ~GLFrameBufferObject();
    bool init(unsigned w, unsigned h);
    void free();
    bool bind();
    void release();
    void blit(GLuint target = 0);
    void blit(const Vector2i &pos, GLuint target = 0);
    inline bool ready() const { return fbo_ != 0; }

    bool init_color(GLint internalformat = GL_RGBA8, GLenum format = GL_RGBA, GLenum type = GL_UNSIGNED_BYTE,
                    GLint minMagFilter = GL_NEAREST);

    bool attach_color(GLuint texID, bool ownTexture = false, GLenum target = GL_TEXTURE_2D);

    bool init_depth();

    bool attach_depth(GLuint texID, bool ownTexture = false, GLenum target = GL_TEXTURE_2D);

    inline GLuint get_fbo() const { return fbo_; }
    inline GLuint get_color_texture() const { return color_texture_; }
    inline GLuint get_depth_texture() const { return depth_texture_; }

    //! Returns width
    inline unsigned get_width() const { return width_; }
    //! Returns height
    inline unsigned get_height() const { return height_; }

    Vector2i get_size() const { return {width_, height_}; }
    int get_samples() const { return samples_; }
    void debug_color_depth();
    void debug_color();
    void debug_depth();

protected: // methods
    //! Deletes/releases the color texture
    void delete_color_texture();

    //! Deletes/releases the depth texture
    void delete_depth_texture();

protected: // members
    //! FBO validity
    bool is_valid_;

    //! Width
    unsigned width_;
    //! Height
    unsigned height_;

    //! Depth texture GL ID
    GLuint depth_texture_;

    //! Whether the depth texture is owned by this FBO or not
    bool own_depth_texture_;

    //! Color texture GL ID
    GLuint color_texture_;

    //! Whether the color texture is owned by this FBO or not
    bool own_color_texture_;

    //! ID
    GLuint fbo_;

    int samples_;
};
} // end h2o
