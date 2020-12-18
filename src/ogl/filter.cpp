/*!
 * \file filter.cpp
 *
 * \author Han
 * \date 2016/11/30
 *
 * fbo 的后处理
 */

#include <ogl/filter.h>

#include <QFile>

namespace h2o {

std::string GLFilter::get_resource_string(const QString &name) {
    QFile file(name);
    file.open(QFile::ReadOnly | QFile::Text);
    QTextStream text(&file);
    QString data = text.readAll();
    return data.toStdString();
}

BilateralFilter::BilateralFilter()
    : GLFilter("Bilateral smooth"), width_(0), height_(0), half_spatial_size_(0), spatial_sigma_(0), depth_sigma_(0),
      damping_pixel_dist_(64, 0), screen_width_(0), screen_height_(0) {
    set_param(2, 2.0f, 0.4f);
}

GLFilter *BilateralFilter::clone() const {
    BilateralFilter *filter = new BilateralFilter();
    // copy parameters
    filter->set_param(half_spatial_size_, spatial_sigma_, depth_sigma_);
    filter->set_screen_size(screen_width_, screen_height_);

    return filter;
}

BilateralFilter::~BilateralFilter() { reset(); }

void BilateralFilter::reset() {
    shader_.free();
    if (fbo_.ready()) {
        fbo_.free();
    }
    width_ = height_ = 0;
}

bool BilateralFilter::init(unsigned width, unsigned height) {
    if (width == 0 || height == 0) {
        return false;
    }

    set_valid(false);

    if (!fbo_.init(static_cast<unsigned>(width), static_cast<unsigned>(height))) {
        reset();
        return false;
    }

    if (!fbo_.bind()) {
        return false;
    }

    if (!fbo_.init_color(GL_RGB, GL_RGB, GL_FLOAT)) {
        return false;
    }
    fbo_.release();

    std::string bilateral_vert = get_resource_string(QStringLiteral(":/shaders/shaders/bilateral.vert"));
    std::string bilateral_frag = get_resource_string(QStringLiteral(":/shaders/shaders/bilateral.frag"));
    if (!shader_.init("bilateral filter", bilateral_vert, bilateral_frag)) {
        reset();
        return false;
    }

    update_shader(0, 0, width, height, shader_);

    CHECK(glGetError() == GL_NO_ERROR) << "Opengl functions failed";
    width_ = width;
    height_ = height;

    set_valid(true);
    return true;
}

void BilateralFilter::set_param(unsigned halfSpatialSize, float spatialSigma, float depthSigma) {
    half_spatial_size_ = std::min<unsigned>(halfSpatialSize, KERNEL_MAX_HALF_SIZE);
    spatial_sigma_ = spatialSigma;
    depth_sigma_ = depthSigma;

    update_damping_table();
}

void BilateralFilter::shade(GLuint texDepth, GLuint texColor, ViewportParameters &parameters) {
    if (!is_valid()) {
        return;
    }

    if (!fbo_.ready()) {
        return;
    }

    Matrix4f mvp = compute_opengl_matrices(screen_width_, screen_height_);
    //	HORIZONTAL
    fbo_.bind();

    shader_.bind();
    shader_.setUniform("s2_I", 0); // image to blur
    shader_.setUniform("s2_D", 1); // image to modulate filter
    shader_.setUniform("SX", static_cast<float>(width_));
    shader_.setUniform("SY", static_cast<float>(height_));
    shader_.setUniform("NHalf", half_spatial_size_);
    shader_.setUniform1fv("DistCoefs", damping_pixel_dist_);
    shader_.setUniform("SigmaDepth", depth_sigma_);
    shader_.setUniform("mvp", mvp);

    // Texture 1 --> 2D
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texDepth);
    // Texture 0 --> 2D
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texColor);

    update_shader(0, 0, width_, height_, shader_);
    shader_.drawIndexed(GL_TRIANGLES, 0, 2);

    glBindTexture(GL_TEXTURE_2D, 0);

    // Texture 1 --> 2D
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);

    fbo_.release();

    // restore GL_TEXTURE_0 by default
    glActiveTexture(GL_TEXTURE0);
}

void BilateralFilter::update_damping_table() {
    assert(half_spatial_size_ <= KERNEL_MAX_HALF_SIZE);

    // constant quotient
    float q = half_spatial_size_ * spatial_sigma_;
    q = 2 * (q * q);

    for (unsigned c = 0; c <= half_spatial_size_; c++) {
        for (unsigned d = 0; d <= half_spatial_size_; d++) {
            // pixel distance based damping
            damping_pixel_dist_[c * (half_spatial_size_ + 1) + d] = exp((c * c + d * d) / (-q));
        }
    }
}

void BilateralFilter::update_shader(int x, int y, int w, int h, GLShader &shader) {
    MatrixXf vertices;
    MatrixXf texcoods;
    MatrixXi indicies;

    vertices.resize(3, 4);
    indicies.resize(3, 2);
    texcoods.resize(2, 4);

    vertices.col(0) = Vector3f(x, y, 0);
    vertices.col(1) = Vector3f(x + w, y, 0);
    vertices.col(2) = Vector3f(x + w, y + h, 0);
    vertices.col(3) = Vector3f(x, y + h, 0);
    indicies.col(0) = Vector3i(0, 1, 2);
    indicies.col(1) = Vector3i(2, 3, 0);
    texcoods.col(0) = Vector2f(0.0f, 0.0f);
    texcoods.col(1) = Vector2f(1.0f, 0.0f);
    texcoods.col(2) = Vector2f(1.0f, 1.0f);
    texcoods.col(3) = Vector2f(0.0f, 1.0f);

    shader.bind();
    if (shader.hasAttrib("position")) {
        shader.freeAttrib("position");
    }
    shader.uploadAttrib("position", vertices);

    if (shader.hasAttrib("texcoord")) {
        shader.freeAttrib("texcoord");
    }
    shader.uploadAttrib("texcoord", texcoods);

    if (shader.hasAttrib("indices")) {
        shader.freeAttrib("indices");
    }
    shader.uploadIndices(indicies);
}

Matrix4f BilateralFilter::compute_opengl_matrices(int w, int h) {
    //     Matrix4f model, view, proj;
    //     model.setIdentity();
    //     view = lookAt({ (float)w / 2, (float)h / 2, 1.0f }, { (float)w / 2, (float)h / 2, 0.0f }, { 0.0f, 1.0f, 0.0f
    //     });
    //     proj = ortho(0.0f, (float)w, 0.0f, (float)h, 0.0f, 1.0f);
    //     return proj * view * model;
    return ortho(0.0f, (float)w, 0.0f, (float)h, 0.0f, 1.0f);
    ;
}

// For MSVC
#ifndef M_PI
#define M_PI 3.141592653589793238462643
#endif

EDLFilter::EDLFilter()
    : GLFilter("EyeDome Lighting (disable normals and increase points size for a better result!)"), screen_width_(0),
      screen_height_(0), exp_scale_(10.0f) {

    // smoothing filter for full resolution
    bilateral_filters_[0].enabled = false;
    bilateral_filters_[0].half_size = 1;
    bilateral_filters_[0].sigma = 1.0f;
    bilateral_filters_[0].sigmaz = 0.2f;

    // smoothing filter for half resolution
    bilateral_filters_[1].enabled = true;
    bilateral_filters_[1].half_size = 2;
    bilateral_filters_[1].sigma = 2.0f;
    bilateral_filters_[1].sigmaz = 0.4f;

    // smoothing filter for quarter resolution
    bilateral_filters_[2].enabled = true;
    bilateral_filters_[2].half_size = 2;
    bilateral_filters_[2].sigma = 2.0f;
    bilateral_filters_[2].sigmaz = 0.4f;

    set_light(static_cast<float>(M_PI / 2), static_cast<float>(M_PI / 2));

    memset(neighbours_, 0, sizeof(float) * 8 * 2);
    for (unsigned c = 0; c < 8; c++) {
        neighbours_[2 * c] = static_cast<float>(cos(static_cast<double>(c) * M_PI / 4));
        neighbours_[2 * c + 1] = static_cast<float>(sin(static_cast<double>(c) * M_PI / 4));
    }
}

EDLFilter::~EDLFilter() { reset(); }

GLFilter *EDLFilter::clone() const {
    EDLFilter *filter = new EDLFilter();

    // copy parameters (only those that can be changed by the user!)
    filter->set_strength(exp_scale_);
    filter->light_dir_[0] = light_dir_[0];
    filter->light_dir_[1] = light_dir_[1];
    filter->light_dir_[2] = light_dir_[2];

    return filter;
}

void EDLFilter::reset() {
    for (unsigned i = 0; i < FBO_COUNT; ++i) {
        if (fbos_[i].ready()) {
            fbos_[i].free();
        }

        bilateral_filters_[i].filter.reset();
    }

    if (fbo_mix_.ready()) {
        fbo_mix_.free();
    }

    edl_shader_.free();
    mix_shader_.free();

    screen_width_ = screen_height_ = 0;
}

bool EDLFilter::init(unsigned width, unsigned height) { return init(width, height, GL_RGBA, GL_LINEAR); }

bool EDLFilter::init(unsigned width, unsigned height, GLenum internalFormat, GLenum minMagFilter) {
    if (width == 0 || height == 0) {
        return false;
    }
    set_valid(false);

    for (unsigned i = 0; i < FBO_COUNT; ++i) {
        unsigned scale = (1 << i);
        unsigned w = width / scale;
        unsigned h = height / scale;

        GLFrameBufferObject &fbo = fbos_[i];
        if (!fbo.init(w, h) || !fbo.init_color(internalFormat, GL_RGBA, GL_FLOAT, minMagFilter)) {
            reset();
            return false;
        }

        if (bilateral_filters_[i].enabled) {
            if (bilateral_filters_[i].filter.init(w, h)) {
                bilateral_filters_[i].filter.set_screen_size(width, height);
            } else {
                bilateral_filters_[i].filter.reset();
                bilateral_filters_[i].enabled = false;
            }
        } else {
            bilateral_filters_[i].filter.reset();
        }
    }

    if (!fbo_mix_.init(width, height)) {
        reset();
        return false;
    }
    fbo_mix_.init_color(internalFormat, GL_RGBA, GL_FLOAT);

    edl_shader_.free();

    std::string edl_shade_vert = get_resource_string(QStringLiteral(":/shaders/shaders/edl-shade.vert"));
    std::string edl_shade_frag = get_resource_string(QStringLiteral(":/shaders/shaders/edl-shade.frag"));
    if (!edl_shader_.init("EDL Shade", edl_shade_vert, edl_shade_frag)) {
        reset();
        return false;
    }

    mix_shader_.free();
    std::string edl_mix_vert = get_resource_string(QStringLiteral(":/shaders/shaders/edl-mix.vert"));
    std::string edl_mix_frag = get_resource_string(QStringLiteral(":/shaders/shaders/edl-mix.frag"));
    if (!mix_shader_.init("EDL Mix", edl_mix_vert, edl_mix_frag)) {
        reset();
        return false;
    }

    CHECK_OPENGL_ERROR();

    screen_width_ = width;
    screen_height_ = height;

    set_valid(true);

    return true;
}

void EDLFilter::shade(GLuint texDepth, GLuint texColor, ViewportParameters &parameters) {
    if (!is_valid()) {
        return;
    }

    if (screen_width_ < 4 || screen_height_ < 4) {
        return;
    }

    // perspective mode
    int perspectiveMode = parameters.perspective_mode ? 1 : 0;
    // light-balancing based on the current zoom (for ortho. mode only)
    float lightMod =
        perspectiveMode ? 3.0f : static_cast<float>(sqrt(2 * std::max<double>(parameters.zoom, 0.7))); // 1.41 ~ sqrt(2)

    // we must use corner-based screen coordinates
    Matrix4f mvp = compute_opengl_matrices(screen_width_, screen_height_);

    for (unsigned i = 0; i < FBO_COUNT; ++i) {
        GLFrameBufferObject &fbo = fbos_[i];
        unsigned scale = (1 << i); // 1, 2, 4
        Vector3f lighdir = {light_dir_[0], light_dir_[1], light_dir_[2]};
        std::vector<float> neighbors(std::begin(neighbours_), std::end(neighbours_));

        fbo.bind();
        edl_shader_.bind();

        edl_shader_.setUniform("s1_color", 1);
        edl_shader_.setUniform("s2_depth", 0);
        edl_shader_.setUniform("Sx", static_cast<float>(screen_width_));
        edl_shader_.setUniform("Sy", static_cast<float>(screen_height_));
        edl_shader_.setUniform("Zoom", lightMod);
        edl_shader_.setUniform("PerspectiveMode", perspectiveMode);
        edl_shader_.setUniform("Pix_scale", static_cast<float>(scale));
        edl_shader_.setUniform("Exp_scale", exp_scale_);
        edl_shader_.setUniform("Zm", static_cast<float>(parameters.znear));
        edl_shader_.setUniform("ZM", static_cast<float>(parameters.zfar));
        edl_shader_.setUniform("Light_dir", lighdir);
        edl_shader_.setUniform2fv("Neigh_pos_2D", neighbors);
        edl_shader_.setUniform("mvp", mvp);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texColor);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texDepth);

        update_shader(0, 0, screen_width_ / scale, screen_height_ / scale, edl_shader_);

        edl_shader_.drawIndexed(GL_TRIANGLES, 0, 2);

        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);

        fbo.release();

        // smooth the result
        BilateralFilterDesc &bl = bilateral_filters_[i];
        if (bl.enabled) {
            bl.filter.set_param(bl.half_size, bl.sigma, bl.sigmaz);
            bl.filter.shade(texDepth, fbo.get_color_texture(), parameters);
            CHECK_OPENGL_ERROR();
        }
    }

    //***	COMPOSITING		***/

    {
        fbo_mix_.bind();
        mix_shader_.bind();
        mix_shader_.setUniform("s2_I1", 0);
        mix_shader_.setUniform("s2_I2", 1);
        mix_shader_.setUniform("s2_I4", 2);
        mix_shader_.setUniform("s2_D", 3);
        mix_shader_.setUniform("A0", 1.0f);
        mix_shader_.setUniform("A1", 0.5f);
        mix_shader_.setUniform("A2", 0.25f);
        mix_shader_.setUniform("absorb", 1);
        mix_shader_.setUniform("mvp", mvp);

        GLuint texCol0 =
            bilateral_filters_[0].enabled ? bilateral_filters_[0].filter.get_texture() : fbos_[0].get_color_texture();
        GLuint texCol1 =
            bilateral_filters_[1].enabled ? bilateral_filters_[1].filter.get_texture() : fbos_[1].get_color_texture();
        GLuint texCol2 =
            bilateral_filters_[2].enabled ? bilateral_filters_[2].filter.get_texture() : fbos_[2].get_color_texture();

        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, texDepth);

        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, texCol2);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texCol1);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texCol0);

        update_shader(0, 0, screen_width_, screen_height_, mix_shader_);
        mix_shader_.drawIndexed(GL_TRIANGLES, 0, 2);

        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, 0);

        fbo_mix_.release();
    }

    // restore GL_TEXTURE_0 by default
    glActiveTexture(GL_TEXTURE0);
    CHECK_OPENGL_ERROR();
}

GLuint EDLFilter::get_texture() { return (fbo_mix_.ready() ? fbo_mix_.get_color_texture() : 0); }

void EDLFilter::set_light(float theta_rad, float phi_rad) {
    light_dir_[0] = sin(phi_rad) * cos(theta_rad);
    light_dir_[1] = cos(phi_rad);
    light_dir_[2] = sin(phi_rad) * sin(theta_rad);
}

void EDLFilter::blit(GLuint target) {
    if (fbo_mix_.ready()) {
        fbo_mix_.blit(target);
    }
}

void EDLFilter::update_shader(int x, int y, int w, int h, GLShader &shader) {
    MatrixXf vertices;
    MatrixXf texcoods;
    MatrixXi indicies;

    vertices.resize(3, 4);
    indicies.resize(3, 2);
    texcoods.resize(2, 4);

    vertices.col(0) = Vector3f(x, y, 0);
    vertices.col(1) = Vector3f(x + w, y, 0);
    vertices.col(2) = Vector3f(x + w, y + h, 0);
    vertices.col(3) = Vector3f(x, y + h, 0);
    indicies.col(0) = Vector3i(0, 1, 2);
    indicies.col(1) = Vector3i(2, 3, 0);
    texcoods.col(0) = Vector2f(0.0f, 0.0f);
    texcoods.col(1) = Vector2f(1.0f, 0.0f);
    texcoods.col(2) = Vector2f(1.0f, 1.0f);
    texcoods.col(3) = Vector2f(0.0f, 1.0f);

    shader.bind();
    if (shader.hasAttrib("position")) {
        shader.freeAttrib("position");
    }
    shader.uploadAttrib("position", vertices);

    if (shader.hasAttrib("texcoord")) {
        shader.freeAttrib("texcoord");
    }
    shader.uploadAttrib("texcoord", texcoods);

    if (shader.hasAttrib("indices")) {
        shader.freeAttrib("indices");
    }
    shader.uploadIndices(indicies);
}

Matrix4f EDLFilter::compute_opengl_matrices(int w, int h) { return ortho(0.0f, (float)w, 0.0f, (float)h, 0.0f, 1.0f); }

} // end h2o
