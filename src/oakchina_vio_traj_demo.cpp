#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>
#include <mutex>
#include <cstring>
#include <sstream>
#include <thread>
#include <chrono>

#include "carina_a1088.h"
#include <opencv2/opencv.hpp>

#include "trajectory_viewer_opencv.hpp"

#ifndef WIN32
#include <sys/ioctl.h>
#include <termios.h>

static bool kbhit_linux()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);
    return byteswaiting > 0;
}
#else
#include <conio.h>
#endif

// ------------------------------
// Thread-safe latest buffers
// ------------------------------
static std::mutex image_mtx;
static cv::Mat left_image;
static cv::Mat right_image;
static cv::Mat left_image1;
static cv::Mat right_image1;
static double image_ts = -1.0;

static std::mutex pose_mtx;
static float pose_data[32] = {0.f};
static double pose_ts = -1.0;

// optional: print throttling
static double last_print_ts = -1.0;

// ------------------------------
// SDK callbacks (MUST be light)
// ------------------------------
void CarinaA1088PoseCallBack(float *pose, double ts)
{
    std::lock_guard<std::mutex> lk(pose_mtx);
    if (pose != nullptr) {
        std::memcpy(pose_data, pose, sizeof(float) * 32);
        pose_ts = ts;
    }
}

void CarinaA1088VsyncCallBack(double /*ts*/) {}

void CarinaA1088ImuCallBack(float * /*imu*/, double /*ts*/) {}

void CarinaA1088CameraCallBack(char *left, char *right, char *left1, char *right1,
                               double ts, int w, int h)
{
    std::lock_guard<std::mutex> lk(image_mtx);

    // NOTE: keep callback light: only allocate+memcpy
    if (left != nullptr) {
        left_image = cv::Mat(h, w, CV_8UC1);
        std::memcpy(left_image.data, left, static_cast<size_t>(w) * static_cast<size_t>(h));
    }
    if (right != nullptr) {
        right_image = cv::Mat(h, w, CV_8UC1);
        std::memcpy(right_image.data, right, static_cast<size_t>(w) * static_cast<size_t>(h));
    }
    if (left1 != nullptr) {
        left_image1 = cv::Mat(h, w, CV_8UC1);
        std::memcpy(left_image1.data, left1, static_cast<size_t>(w) * static_cast<size_t>(h));
    }
    if (right1 != nullptr) {
        right_image1 = cv::Mat(h, w, CV_8UC1);
        std::memcpy(right_image1.data, right1, static_cast<size_t>(w) * static_cast<size_t>(h));
    }
    image_ts = ts;
}

void CarinaA1088PointsCallBack(carina_points & /*points*/, double /*ts*/) {}

void CarinaA1088EventCallBack(const unsigned char /*uc_event*/) {}

// ------------------------------
// Helpers
// ------------------------------
static bool load_file_to_string(const std::string& path, std::string& out)
{
    std::ifstream ifs(path, std::ios::in);
    if (!ifs.is_open()) return false;
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    out = buffer.str();
    return true;
}

static void decode_Twb_column_major(const float pose32[32],
                                   cv::Vec3f& t_wb,
                                   cv::Matx33f& R_wb)
{
    // pose[0..15] is 4x4 column-major Twb
    // translation
    t_wb = cv::Vec3f(pose32[12], pose32[13], pose32[14]);

    // rotation (convert to row-major Matx33f)
    R_wb = cv::Matx33f(
        pose32[0],  pose32[4],  pose32[8],
        pose32[1],  pose32[5],  pose32[9],
        pose32[2],  pose32[6],  pose32[10]
    );
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage:\n"
                  << "  .\\oakchina_vio_traj_demo ..\\custom_config.yaml ..\\database.bin [image_show=0/1]\n";
        return 0;
    }

    const std::string config_file_path = argv[1];
    char* database_file_path = argv[2];

    bool image_show = true;
    if (argc >= 4) {
        std::string s = argv[3];
        if (s == "0") image_show = false;
    }

    std::string custom_config;
    if (!load_file_to_string(config_file_path, custom_config)) {
        std::cout << "Please set the correct custom_config.yaml path\n";
        return 0;
    }

    // ------------------------------
    // SDK init/start
    // ------------------------------
    carina_a1088_init(const_cast<char*>(custom_config.c_str()), database_file_path);

    carina_a1088_start(
        CarinaA1088PoseCallBack,
        CarinaA1088VsyncCallBack,
        CarinaA1088ImuCallBack,
        CarinaA1088CameraCallBack,
        CarinaA1088PointsCallBack,
        CarinaA1088EventCallBack
    );

    carina_a1088_resume();

    // print basic info (optional)
    {
        std::string cam_param = carina_a1088_get_cam_param();
        std::cout << "cam_param:\n" << cam_param << "\n------------------------\n";
        std::string sn = carina_a1088_get_sn();
        std::cout << "sn:\n" << sn << "\n------------------------\n";
        std::string sdk_version = carina_a1088_get_sdk_version();
        std::cout << "sdk_version:\n" << sdk_version << "\n------------------------\n";
        std::string firmware_version = carina_a1088_get_firmware_version();
        std::cout << "firmware_version:\n" << firmware_version << "\n------------------------\n";
    }

    // ------------------------------
    // Windows / viewers
    // ------------------------------
    TrajectoryViewerOpenCV viewer(960, 720);

    cv::Mat left_show, right_show, left1_show, right1_show;

    double last_pose_consumed_ts = -1.0;

    // Optional: downsample settings (can tune)
    const double min_dt_sec = 0.010;   // 10ms
    const float  min_dist_m = 0.01f;   // 1cm
    const size_t max_points = 20000;

    // ------------------------------
    // Main loop
    // ------------------------------
    for (;;)
    {
        // 1) fetch latest pose (non-blocking heavy work)
        double pose_ts_local = -1.0;
        float pose_copy[32] = {0.f};
        {
            std::lock_guard<std::mutex> lk(pose_mtx);
            if (pose_ts > 0) {
                pose_ts_local = pose_ts;
                std::memcpy(pose_copy, pose_data, sizeof(float) * 32);
                // do NOT reset pose_ts here; keep latest pose available
            }
        }

        if (pose_ts_local > 0 && pose_ts_local != last_pose_consumed_ts) {
            cv::Vec3f t_wb;
            cv::Matx33f R_wb;
            decode_Twb_column_major(pose_copy, t_wb, R_wb);

            // acceptance requirement: console prints xyz changing
            if (last_print_ts < 0 || (pose_ts_local - last_print_ts) > 0.2) {
                std::cout << "pose ts: " << pose_ts_local
                          << " xyz: " << t_wb[0] << "\t" << t_wb[1] << "\t" << t_wb[2] << "\n";
                last_print_ts = pose_ts_local;
            }

            // trajectory sampling
            viewer.appendPoint(t_wb, pose_ts_local, min_dt_sec, min_dist_m, max_points);
            last_pose_consumed_ts = pose_ts_local;
        }

        // 2) fetch latest images for display
        if (image_show) {
            double img_ts_local = -1.0;
            cv::Mat l, r, l1, r1;
            {
                std::lock_guard<std::mutex> lk(image_mtx);
                if (image_ts > 0) {
                    img_ts_local = image_ts;
                    if (!left_image.empty())  l = left_image.clone();
                    if (!right_image.empty()) r = right_image.clone();
                    if (!left_image1.empty())  l1 = left_image1.clone();
                    if (!right_image1.empty()) r1 = right_image1.clone();
                    image_ts = -1.0; // consume
                }
            }
            if (img_ts_local > 0) {
                left_show = std::move(l);
                right_show = std::move(r);
                left1_show = std::move(l1);
                right1_show = std::move(r1);
            }

            if (!left_show.empty())  cv::imshow("left", left_show);
            if (!right_show.empty()) cv::imshow("right", right_show);
            if (!left1_show.empty())  cv::imshow("left1", left1_show);
            if (!right1_show.empty()) cv::imshow("right1", right1_show);
        }

        // 3) render trajectory window
        viewer.renderFrame();

        // 4) key events / exit conditions
        int key = cv::waitKey(1);
        viewer.handleKey(key);
        if (key == 'q' || key == 27) { // q or ESC
            break;
        }

        // Also exit if window is closed
        if (!viewer.isVisible()) {
            break;
        }

#ifndef WIN32
        if (kbhit_linux()) {
            int c = fgetc(stdin);
            if (c == 'q') break;
        }
#else
        if (_kbhit()) {
            int c = _getch();
            if (c == 'q') break;
        }
#endif

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cv::destroyAllWindows();
    carina_a1088_pause();
    carina_a1088_stop();
    carina_a1088_release();
    return EXIT_SUCCESS;
}
