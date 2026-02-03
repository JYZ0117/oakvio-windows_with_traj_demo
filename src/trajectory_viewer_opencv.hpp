#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <string>

class TrajectoryViewerOpenCV
{
public:
    TrajectoryViewerOpenCV(int width, int height);

    void setTitle(const std::string& title);

    // Append a 3D point in world coordinates
    // Sampling rules: append only if dt >= min_dt_sec OR dist >= min_dist_m
    // Cap: keep at most max_points points.
    void appendPoint(const cv::Vec3f& pw, double ts,
                     double min_dt_sec, float min_dist_m,
                     size_t max_points);

    // Render the trajectory window once (non-blocking)
    void renderFrame();

    // Handle keypress (pass return value of cv::waitKey)
    void handleKey(int key);

    // Check whether the window is visible (user closed it)
    bool isVisible() const;

    // Save CSV (ts,x,y,z)
    bool saveCsv(const std::string& path) const;

    // Clear trajectory
    void clear();

private:
    // Mouse + key handlers
    static void MouseCallback(int event, int x, int y, int flags, void* userdata);
    void onMouse(int event, int x, int y, int flags);

    // Rendering helpers
    void drawAxes(cv::Mat& canvas) const;
    void drawGrid(cv::Mat& canvas) const;
    bool projectPoint(const cv::Vec3f& pw, cv::Point& uv, float& z_cam) const;

    // Camera
    void resetView();
    void updateCameraBasis(cv::Vec3f& camPos, cv::Vec3f& right, cv::Vec3f& up, cv::Vec3f& fwd) const;

private:
    int width_;
    int height_;
    std::string title_;

    // Trajectory points
    mutable std::mutex mtx_;
    std::vector<cv::Vec3f> points_;
    std::vector<double> ts_;
    bool follow_last_ = true;

    // Camera parameters
    float yaw_ = 0.0f;        // radians
    float pitch_ = -0.35f;    // radians
    float radius_ = 5.0f;     // meters (virtual)
    cv::Vec3f center_ = cv::Vec3f(0,0,0);
    float fov_deg_ = 60.0f;

    // Mouse state
    bool ldrag_ = false;
    bool rdrag_ = false;
    cv::Point last_mouse_{0,0};

    // Interaction tuning
    float rot_sens_ = 0.005f;
    float pan_sens_ = 0.002f;
    float zoom_sens_ = 0.1f;
};
