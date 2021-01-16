/*!
 * \file frame_buffer_object.cpp
 *
 * \author Han
 * \date 2016/11/30
 *
 * fbo 封装
 */

#include <ogl/frame_buffer_object.h>
#include <ogl/shader.h>

namespace h2o {

GLFrameBufferObject::GLFrameBufferObject(int samples)
    : is_valid_(false), width_(0), height_(0), depth_texture_(0), own_depth_texture_(false), color_texture_(0),
      own_color_texture_(false), fbo_(0) {
    initializeOpenGLFunctions();
    samples_ = samples;
}

GLFrameBufferObject::~GLFrameBufferObject() { free(); }

void GLFrameBufferObject::free() {
    if (!is_valid_) {
        return;
    }

    delete_depth_texture();
    delete_color_texture();

    if (fbo_ != 0) {
        glDeleteFramebuffers(1, &fbo_);
        fbo_ = 0;
    }
    is_valid_ = false;
    width_ = height_ = 0;
}

bool GLFrameBufferObject::init(unsigned w, unsigned h) {
    if (!is_valid_) {

    } else {
        // to support reinit
        free();
    }

    width_ = w;
    height_ = h;

    // create a framebuffer object
    glGenFramebuffers(1, &fbo_);
    CHECK_OPENGL_ERROR();
    is_valid_ = true;

    return fbo_ != 0;
}

bool GLFrameBufferObject::bind() {
    if (is_valid_ && fbo_ != 0) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        if (samples_ > 1) {
            glEnable(GL_MULTISAMPLE);
        }
        return true;
    } else {
        return false;
    }
    CHECK_OPENGL_ERROR();
}

void GLFrameBufferObject::release() {
    if (is_valid_ && fbo_ != 0) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if (samples_ > 1) {
            glDisable(GL_MULTISAMPLE);
        }
    }
    CHECK_OPENGL_ERROR();
}

void GLFrameBufferObject::blit(GLuint target) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    CHECK_OPENGL_ERROR();

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target);
    CHECK_OPENGL_ERROR();

    glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT,
                      GL_NEAREST);

    CHECK_OPENGL_ERROR();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GLFrameBufferObject::blit(const Vector2i &pos, GLuint target) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target);
    glDrawBuffer(GL_BACK);
    glBlitFramebuffer(0, 0, width_, height_, pos.x(), pos.y(), pos.x() + width_, pos.y() + height_,
                      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

std::tuple<cv::Mat, cv::Mat> GLFrameBufferObject::debug_color_depth() {
    cv::Mat image_color = cv::Mat::zeros(height_, width_, CV_8UC4);
    cv::Mat image_depth = cv::Mat::zeros(height_, width_, CV_32F);

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glReadPixels(0, 0, width_, height_, GL_BGRA, GL_UNSIGNED_BYTE, image_color.data);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glReadPixels(0, 0, width_, height_, GL_DEPTH_COMPONENT, GL_FLOAT, image_depth.data);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    CHECK_OPENGL_ERROR();

    return std::make_tuple(image_color, image_depth);
}

cv::Mat GLFrameBufferObject::debug_color() {
    cv::Mat image_color = cv::Mat::zeros(height_, width_, CV_8UC4);

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glReadPixels(0, 0, width_, height_, GL_BGRA, GL_UNSIGNED_BYTE, image_color.data);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    return image_color;
}

cv::Mat GLFrameBufferObject::debug_depth() {
    cv::Mat image_depth = cv::Mat::zeros(height_, width_, CV_32F);

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glReadPixels(0, 0, width_, height_, GL_DEPTH_COMPONENT, GL_FLOAT, image_depth.data);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    return image_depth;
}

void GLFrameBufferObject::delete_color_texture() {
    if (is_valid_ && own_color_texture_ && glIsTexture(color_texture_)) {
        glDeleteTextures(1, &color_texture_);
    }
    color_texture_ = 0;
    own_color_texture_ = false;
}

void GLFrameBufferObject::delete_depth_texture() {
    if (is_valid_ && own_depth_texture_ && glIsTexture(depth_texture_)) {
        glDeleteTextures(1, &depth_texture_);
    }
    depth_texture_ = 0;
    own_depth_texture_ = false;
}

bool GLFrameBufferObject::init_color(GLint internalformat, GLenum format, GLenum type, GLint minMagFilter) {
    if (!is_valid_ || fbo_ == 0) {
        assert(false);
        return false;
    }
    if (!bind()) {
        return false;
    }

    GLenum target = GL_TEXTURE_2D;
    if (samples_ > 1) {
        target = GL_TEXTURE_2D_MULTISAMPLE;
    }

    // create the new texture
    GLuint texID = 0;
    glGenTextures(1, &texID);
    glBindTexture(target, texID);

    if (samples_ > 1) {
        glTexImage2DMultisample(target, samples_, internalformat, width_, height_, false);
    } else {
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, minMagFilter);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, minMagFilter);
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(target, 0, internalformat, width_, height_, 0, format, type, 0);
    }
    CHECK_OPENGL_ERROR();

    glBindTexture(target, 0);
    if (attach_color(texID, true, target)) {
        return true;
    } else {
        glDeleteTextures(1, &texID);
        return false;
    }
}

bool GLFrameBufferObject::attach_color(GLuint texID, bool ownTexture /*=false*/, GLenum target /*=GL_TEXTURE_2D*/) {
    if (!is_valid_ || fbo_ == 0) {
        assert(false);
        return false;
    }

    if (!glIsTexture(texID)) {
        // error or simple warning?
        assert(false);
        // return false;
    }

    if (!bind()) {
        return false;
    }

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, target, texID, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    release();

    bool success = true;
    switch (status) {
    case GL_FRAMEBUFFER_COMPLETE:
        // remove the previous texture (if any)
        delete_color_texture();
        // save the new one
        color_texture_ = texID;
        own_color_texture_ = ownTexture;
        break;

    default:
        throw std::runtime_error("Could not create framebuffer object!");
        success = false;
        break;
    }

    return success;
}

bool GLFrameBufferObject::init_depth() {
    if (!is_valid_ || fbo_ == 0) {
        CHECK(false) << "init failed";
        return false;
    }

    if (!bind()) {
        return false;
    }

    GLenum target = GL_TEXTURE_2D;
    if (samples_ > 1) {
        target = GL_TEXTURE_2D_MULTISAMPLE;
    }

    GLuint texID = 0;
    glGenTextures(1, &texID);
    glBindTexture(target, texID);

    if (samples_ > 1) {
        glTexImage2DMultisample(target, samples_, GL_DEPTH24_STENCIL8, width_, height_, false);
    } else {
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_COMPARE_MODE, GL_NONE);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(target, 0, GL_DEPTH24_STENCIL8, width_, height_, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 0);
    }

    glBindTexture(target, 0);
    if (attach_depth(texID, true, target)) {
        return true;
    } else {
        glDeleteTextures(1, &texID);
        return false;
    }
}

bool GLFrameBufferObject::attach_depth(GLuint texID, bool ownTexture /*=false*/, GLenum target /*=GL_TEXTURE_2D*/) {
    if (!is_valid_ || fbo_ == 0) {
        CHECK(false) << "init failed";
        return false;
    }

    if (!glIsTexture(texID)) {
        // error or simple warning?
        assert(false);
        // return false;
    }

    if (!bind()) {
        return false;
    }

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, target, texID, 0);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    release();

    bool success = true;
    switch (status) {
    case GL_FRAMEBUFFER_COMPLETE:
        // remove the previous texture (if any)
        delete_depth_texture();
        // save the new one
        depth_texture_ = texID;
        own_depth_texture_ = ownTexture;
        break;

    default:
        throw std::runtime_error("Could not create framebuffer object!");
        success = false;
        break;
    }

    return success;
}

} // namespace h2o
