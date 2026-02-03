#include "trajectory_viewer_opencv.hpp"

#include <cmath>
#include <fstream>
#include <algorithm>

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}
static inline cv::Vec3f normalize3(const cv::Vec3f& v) {
    float n = std::sqrt(v.dot(v));
    if (n < 1e-9f) return cv::Vec3f(0,0,0);
    return v * (1.0f / n);
}
static inline cv::Vec3f cross3(const cv::Vec3f& a, const cv::Vec3f& b) {
    return cv::Vec3f(
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    );
}

TrajectoryViewerOpenCV::TrajectoryViewerOpenCV(int width, int height)
: width_(width), height_(height)
{
    title_ = "trajectory_3d";
    resetView();
    cv::namedWindow(title_, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(title_, &TrajectoryViewerOpenCV::MouseCallback, this);
}

void TrajectoryViewerOpenCV::setTitle(const std::string& title)
{
    title_ = title;
    cv::namedWindow(title_, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(title_, &TrajectoryViewerOpenCV::MouseCallback, this);
}

bool TrajectoryViewerOpenCV::isVisible() const
{
#if CV_VERSION_MAJOR >= 3
    double v = cv::getWindowProperty(title_, cv::WND_PROP_VISIBLE);
    return v >= 1.0;
#else
    return true;
#endif
}

void TrajectoryViewerOpenCV::resetView()
{
    yaw_ = 0.0f;
    pitch_ = -0.35f;
    radius_ = 5.0f;
    center_ = cv::Vec3f(0,0,0);
    follow_last_ = true;
}

void TrajectoryViewerOpenCV::clear()
{
    std::lock_guard<std::mutex> lk(mtx_);
    points_.clear();
    ts_.clear();
    center_ = cv::Vec3f(0,0,0);
}

bool TrajectoryViewerOpenCV::saveCsv(const std::string& path) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    std::ofstream ofs(path, std::ios::out);
    if (!ofs.is_open()) return false;
    ofs << "ts,x,y,z\n";
    for (size_t i=0;i<points_.size();++i) {
        ofs << ts_[i] << "," << points_[i][0] << "," << points_[i][1] << "," << points_[i][2] << "\n";
    }
    return true;
}

void TrajectoryViewerOpenCV::appendPoint(const cv::Vec3f& pw, double ts,
                                        double min_dt_sec, float min_dist_m,
                                        size_t max_points)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!points_.empty()) {
        double dt = ts - ts_.back();
        cv::Vec3f d = pw - points_.back();
        float dist = std::sqrt(d.dot(d));

        if (dt < min_dt_sec && dist < min_dist_m) {
            return;
        }
    }
    points_.push_back(pw);
    ts_.push_back(ts);

    if (follow_last_) {
        center_ = pw;
    }

    // cap
    if (points_.size() > max_points) {
        size_t drop = points_.size() - max_points;
        points_.erase(points_.begin(), points_.begin() + static_cast<long>(drop));
        ts_.erase(ts_.begin(), ts_.begin() + static_cast<long>(drop));
    }
}

void TrajectoryViewerOpenCV::MouseCallback(int event, int x, int y, int flags, void* userdata)
{
    auto* self = reinterpret_cast<TrajectoryViewerOpenCV*>(userdata);
    if (self) self->onMouse(event, x, y, flags);
}

void TrajectoryViewerOpenCV::onMouse(int event, int x, int y, int flags)
{
    cv::Point cur(x,y);

    if (event == cv::EVENT_LBUTTONDOWN) {
        ldrag_ = true;
        last_mouse_ = cur;
    } else if (event == cv::EVENT_LBUTTONUP) {
        ldrag_ = false;
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        rdrag_ = true;
        last_mouse_ = cur;
    } else if (event == cv::EVENT_RBUTTONUP) {
        rdrag_ = false;
    } else if (event == cv::EVENT_MOUSEMOVE) {
        cv::Point d = cur - last_mouse_;
        last_mouse_ = cur;

        if (ldrag_) {
            yaw_   += static_cast<float>(d.x) * rot_sens_;
            pitch_ += static_cast<float>(d.y) * rot_sens_;
            pitch_ = clampf(pitch_, -1.50f, 1.50f);
        }
        if (rdrag_) {
            // pan in camera plane
            cv::Vec3f camPos, right, up, fwd;
            updateCameraBasis(camPos, right, up, fwd);
            float scale = radius_ * pan_sens_;
            center_ += (-static_cast<float>(d.x) * scale) * right;
            center_ += ( static_cast<float>(d.y) * scale) * up;
            follow_last_ = false;
        }
    }
#if CV_VERSION_MAJOR >= 3
    else if (event == cv::EVENT_MOUSEWHEEL) {
        // wheel delta in high 16 bits
        int delta = cv::getMouseWheelDelta(flags);
        if (delta > 0) radius_ *= (1.0f - zoom_sens_);
        else if (delta < 0) radius_ *= (1.0f + zoom_sens_);
        radius_ = clampf(radius_, 0.1f, 200.0f);
    }
#endif
}

void TrajectoryViewerOpenCV::updateCameraBasis(cv::Vec3f& camPos,
                                              cv::Vec3f& right,
                                              cv::Vec3f& up,
                                              cv::Vec3f& fwd) const
{
    float cy = std::cos(yaw_);
    float sy = std::sin(yaw_);
    float cp = std::cos(pitch_);
    float sp = std::sin(pitch_);

    // camera position in world
    cv::Vec3f dir(cp * cy, cp * sy, sp);
    camPos = center_ + radius_ * dir;

    // look at center_
    fwd = normalize3(center_ - camPos); // forward
    cv::Vec3f world_up(0,0,1);
    right = cross3(fwd, world_up);
    float rn = std::sqrt(right.dot(right));
    if (rn < 1e-6f) {
        // in case forward ~ world_up, pick another up
        world_up = cv::Vec3f(0,1,0);
        right = cross3(fwd, world_up);
    }
    right = normalize3(right);
    up = normalize3(cross3(right, fwd));
}

bool TrajectoryViewerOpenCV::projectPoint(const cv::Vec3f& pw, cv::Point& uv, float& z_cam) const
{
    cv::Vec3f camPos, right, up, fwd;
    updateCameraBasis(camPos, right, up, fwd);

    cv::Vec3f d = pw - camPos;
    float x = d.dot(right);
    float y = d.dot(up);
    float z = d.dot(fwd);
    z_cam = z;

    if (z <= 1e-3f) return false;

    float fov = fov_deg_ * static_cast<float>(CV_PI) / 180.0f;
    float fx = (0.5f * width_) / std::tan(0.5f * fov);
    float fy = fx;
    float cx = 0.5f * width_;
    float cy = 0.5f * height_;

    int u = static_cast<int>(fx * (x / z) + cx);
    int v = static_cast<int>(fy * (y / z) + cy);

    uv = cv::Point(u, v);
    return (u >= -1000 && u <= width_ + 1000 && v >= -1000 && v <= height_ + 1000);
}

void TrajectoryViewerOpenCV::drawAxes(cv::Mat& canvas) const
{
    // axes from origin
    const float L = 1.0f;

    cv::Point o, px, py, pz;
    float zc;

    bool ok0 = projectPoint(cv::Vec3f(0,0,0), o, zc);
    bool okx = projectPoint(cv::Vec3f(L,0,0), px, zc);
    bool oky = projectPoint(cv::Vec3f(0,L,0), py, zc);
    bool okz = projectPoint(cv::Vec3f(0,0,L), pz, zc);

    if (ok0 && okx) cv::line(canvas, o, px, cv::Scalar(0,0,255), 2, cv::LINE_AA); // X red
    if (ok0 && oky) cv::line(canvas, o, py, cv::Scalar(0,255,0), 2, cv::LINE_AA); // Y green
    if (ok0 && okz) cv::line(canvas, o, pz, cv::Scalar(255,0,0), 2, cv::LINE_AA); // Z blue

    if (okx) cv::putText(canvas, "X", px + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    if (oky) cv::putText(canvas, "Y", py + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 1, cv::LINE_AA);
    if (okz) cv::putText(canvas, "Z", pz + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0), 1, cv::LINE_AA);
}

void TrajectoryViewerOpenCV::drawGrid(cv::Mat& canvas) const
{
    // draw a simple grid on z=0 plane around origin
    const int N = 10;
    const float step = 1.0f;
    const float half = N * step;

    for (int i = -N; i <= N; ++i) {
        float x = i * step;
        cv::Point a,b;
        float zc;
        if (projectPoint(cv::Vec3f(x, -half, 0), a, zc) &&
            projectPoint(cv::Vec3f(x,  half, 0), b, zc)) {
            cv::line(canvas, a, b, cv::Scalar(50,50,50), 1, cv::LINE_AA);
        }
        float y = i * step;
        if (projectPoint(cv::Vec3f(-half, y, 0), a, zc) &&
            projectPoint(cv::Vec3f( half, y, 0), b, zc)) {
            cv::line(canvas, a, b, cv::Scalar(50,50,50), 1, cv::LINE_AA);
        }
    }
}

void TrajectoryViewerOpenCV::handleKey(int key)
{
    if (key == 'r') {
        resetView();
    } else if (key == 'c') {
        clear();
    } else if (key == 'f') {
        follow_last_ = !follow_last_;
    } else if (key == 's') {
        saveCsv("trajectory.csv");
    } else if (key == '+' || key == '=') {
        radius_ *= (1.0f - zoom_sens_);
        radius_ = clampf(radius_, 0.1f, 200.0f);
    } else if (key == '-' || key == '_') {
        radius_ *= (1.0f + zoom_sens_);
        radius_ = clampf(radius_, 0.1f, 200.0f);
    }
}

void TrajectoryViewerOpenCV::renderFrame()
{
    cv::Mat canvas(height_, width_, CV_8UC3, cv::Scalar(15,15,15));

    drawGrid(canvas);
    drawAxes(canvas);

    // snapshot points
    std::vector<cv::Vec3f> pts;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        pts = points_;
    }

    if (pts.size() >= 2) {
        std::vector<cv::Point> uv;
        uv.reserve(pts.size());
        float zc;
        for (const auto& p : pts) {
            cv::Point u;
            if (projectPoint(p, u, zc)) {
                uv.push_back(u);
            } else {
                // push invalid marker
                uv.push_back(cv::Point(-99999, -99999));
            }
        }

        for (size_t i=1; i<uv.size(); ++i) {
            const auto& p0 = uv[i-1];
            const auto& p1 = uv[i];
            if (p0.x < -10000 || p1.x < -10000) continue;
            cv::line(canvas, p0, p1, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
        }

        // draw latest point
        const auto& last = uv.back();
        if (last.x > -10000) {
            cv::circle(canvas, last, 4, cv::Scalar(0,255,255), -1, cv::LINE_AA);
        }
    }

    // HUD text
    {
        std::lock_guard<std::mutex> lk(mtx_);
        std::string s1 = "points: " + std::to_string(points_.size());
        std::string s2 = std::string("mouse: L=rot R=pan wheel=zoom | keys: r reset, c clear, f follow, s save, q/esc quit");
        cv::putText(canvas, s1, cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(230,230,230), 1, cv::LINE_AA);
        cv::putText(canvas, s2, cv::Point(12, height_ - 16), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    }

    cv::imshow(title_, canvas);
}
