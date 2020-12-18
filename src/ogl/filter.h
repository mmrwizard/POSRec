/*!
 * \file filter.h
 *
 * \author Han
 * \date 2016/11/30
 *
 * fbo 的影像后处理
 */
#pragma once

#include <base/common.h>
#include <ogl/camera.h>
#include <ogl/frame_buffer_object.h>
#include <ogl/shader.h>

#include <QOpenGLFunctions_3_3_Core>
#include <QString>

namespace h2o {
class GLFilter : public QOpenGLFunctions_3_3_Core {
public:
    //! Default constructor
    GLFilter(std::string description) : is_valid_(false), description_(description) {
        CHECK(initializeOpenGLFunctions());
    }

    //! Default destructor
    virtual ~GLFilter() {}

    //! Returns filter name
    inline virtual std::string get_description() const { return description_; }

    //! Cloning mechanism
    virtual GLFilter *clone() const = 0;

    //! Initializes GL filter
    /** Must support reinit!
    \param width texture/screen width
    \param height texture/screen height
    \param shadersPath path where shader files are stored
    \param error error string (if an error occurred)
    \return success
    **/
    virtual bool init(unsigned width, unsigned height) = 0;

    //! Minimal set of 3D viewport parameters that can be used by shaders
    struct ViewportParameters {
        //! Default constructor
        ViewportParameters() : zoom(1.0), perspective_mode(false), znear(0.0), zfar(1.0) {}

        //! Zoom
        double zoom;
        //! Whether perspective mode is enabled or not
        bool perspective_mode;
        //! Near clipping plane position (perspective mode only)
        double znear;
        //! Far clipping plane position (perspective mode only)
        double zfar;
    };

    //! Applies filter to texture (depth + color)
    virtual void shade(unsigned depth, unsigned color, ViewportParameters &parameters) = 0;

    //! Returns resulting texture
    virtual unsigned get_texture() = 0;

    virtual void blit(GLuint target) = 0;

    virtual void reset() = 0;

protected: // methods
    //! Sets whether the filter is valid
    inline void set_valid(bool state) { is_valid_ = state; }

    //! Returns whether the filter is valid
    inline bool is_valid() const { return is_valid_; }

    std::string get_resource_string(const QString &name);

protected:
    //! Filter validity
    bool is_valid_;

    //! Filter description
    std::string description_;
};

//! Bilateral filer (shader)
/** See http://en.wikipedia.org/wiki/Bilateral_filter
**/
class BilateralFilter : public GLFilter {
public:
    //! Default constructor
    /** Default parameters:
    - halfSpatialSize = 2
    - spatialSigma = 2.0
    - depthSigma = 0.4
    **/
    BilateralFilter();
    //! Destructor
    virtual ~BilateralFilter();

    //! Resets the filter
    void reset();

    // inherited from ccGlFilter
    virtual GLFilter *clone() const override;
    bool init(unsigned width, unsigned height) override;
    virtual void shade(GLuint depth, GLuint color, ViewportParameters &parameters) override;
    inline virtual GLuint get_texture() override { return fbo_.get_color_texture(); }

    //! Set parameters
    /** \param halfSpatialSize half spatial kernel size (between 1 and 7 - total size will be 2*h+1)
    \param spatialSigma variance of the 'spatial' distribution (Euclidean distance of pixels)
    \param depthSigma variance of the 'depth' distribution (depth difference of pixels)
    **/
    void set_param(unsigned halfSpatialSize, float spatialSigma, float depthSigma);

    //! Sets whether to use the current context (OpenGL) viewport or not
    void set_screen_size(int w, int h) {
        screen_width_ = w;
        screen_height_ = h;
    }

    void debug_color_texture() { fbo_.debug_color(); }

    virtual void blit(GLuint target){};

protected: // methods
    void update_damping_table();

    void update_shader(int x, int y, int w, int h, GLShader &shader);
    Matrix4f compute_opengl_matrices(int w, int h);

protected: // members
    // (KERNEL_MAX_HALF_SIZE+1) * (KERNEL_MAX_HALF_SIZE+1) = 64
    static constexpr int KERNEL_MAX_HALF_SIZE = 7;
    uint32_t width_;
    uint32_t height_;

    GLFrameBufferObject fbo_;
    GLShader shader_;

    //! Half spatial size (kernel width will be 2*h+1)
    uint32_t half_spatial_size_;
    //! Variance of the 'spatial' distribution (Euclidean distance of pixels)
    float spatial_sigma_;
    //! Variance of the 'depth' distribution (depth difference of pixels)
    float depth_sigma_;

    //! 'spatial' distribution (kernel values)
    std::vector<float> damping_pixel_dist_;

    int screen_width_;
    int screen_height_;
};

//!	EyeDome Lighting
class EDLFilter : public GLFilter {
public:
    //! Default constructor
    EDLFilter();
    //! Default destructor
    virtual ~EDLFilter();

    // inherited from ccGlFilter
    virtual GLFilter *clone() const override;
    virtual bool init(unsigned width, unsigned height) override;
    virtual void shade(GLuint texDepth, GLuint texColor, ViewportParameters &parameters) override;
    virtual GLuint get_texture() override;

    //! Resets filter
    void reset();

    //! Inits filter
    bool init(unsigned width, unsigned height, GLenum internalFormat, GLenum minMagFilter);

    //! Sets light direction
    void set_light(float theta_rad, float phi_rad);

    //! Sets strength
    /** \param value strength value (default: 100)
    **/
    inline void set_strength(float value) { exp_scale_ = value; }

    void blit(GLuint target);

protected:
    void update_shader(int x, int y, int w, int h, GLShader &shader);
    Matrix4f compute_opengl_matrices(int w, int h);

private:
    unsigned screen_width_;
    unsigned screen_height_;

    //! Number of FBOs
    static const unsigned FBO_COUNT = 3;

    GLFrameBufferObject fbos_[FBO_COUNT];
    GLShader edl_shader_;

    GLFrameBufferObject fbo_mix_;
    GLShader mix_shader_;

    float neighbours_[8 * 2];
    float exp_scale_;

    //! Bilateral filter descriptor
    struct BilateralFilterDesc {
        BilateralFilter filter;
        unsigned half_size;
        float sigma;
        float sigmaz;
        bool enabled;

        BilateralFilterDesc() : half_size(0), sigma(0), sigmaz(0), enabled(false) {}

        ~BilateralFilterDesc() {}
    };

    //	Bilateral filters (one per FBO at most)
    BilateralFilterDesc bilateral_filters_[FBO_COUNT];

    // Light direction
    float light_dir_[3];
};

} // end h2o
