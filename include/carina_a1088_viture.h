#ifndef CARINA_VIO_CARINA_A1088_VITURE_H
#define CARINA_VIO_CARINA_A1088_VITURE_H
#ifndef _MSC_VER
#define CARINA_A1088_VITURE_EXPORT __attribute__((visibility("default")))
#else
#define CARINA_A1088_VITURE_EXPORT __declspec(dllexport)
#endif
#include "carina_a1088.h"

#ifdef __cplusplus
extern "C" {
#endif

///
/// \param custom_config yaml value
/// \param vocab_file_path database.bin file path
/// \param input(file_descriptor)，设备描述
/// \param input(bus_num)，设备bus
//// \param input(dev_addr)，设备地址
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_init(char *custom_config,
                                                        char *vocab_file_path,
                                                        int file_descriptor = -1,
                                                        int bus_num = -1,
                                                        int dev_addr = -1);

///
/// pose_callback float[32] 0-15 twb(列存储)
/// points_callback float[500]2d-3d max 100 points,timestamp in monotime
/// vsync_callback timestamp in monotime
/// imu_callback float[6] 0-2 acc 3-5 gyro, timestamp in monotime
/// img_callback const char * left_img, const char * right_img, timestamp in
/// monotime return 0:success other:fail

CARINA_A1088_VITURE_EXPORT int
carina_a1088_viture_start(CarinaA1088PoseCallBackType pose_callback,
                          CarinaA1088VsyncCallBackType vsync_callback,
                          CarinaA1088ImuCallBackType imu_callback,
                          CarinaA1088CameraCallBackType img_callback,
                          CarinaA1088PointsCallBackType points_callback,
                          CarinaA1088EventCallBackType event_callback);

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_stop();

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_release();

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_pause();

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_resume();

/// read sn, need to free memory by caller
/// \return success:sn, fail:null
CARINA_A1088_VITURE_EXPORT char *carina_a1088_viture_get_sn();

/// read camera parameters, need to free memory by caller
/// \return success:cam_param, fail:null
CARINA_A1088_VITURE_EXPORT char *carina_a1088_viture_get_cam_param();

///
//// \param output(config)
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT char *carina_a1088_viture_get_config();

///
/// \param pose twb  列存储 size 16
/// \param predicttime predict seconds, in monotime for system
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_get_imu_pose(float *pose,
                                                                double predicttime);

///
/// \param pose 32 twb列存储16 ,velocity 3, angular_velocity 3
/// \param timestamp predict absolute seconds, use imu time axis
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_get_gl_pose(float *pose, double predicttime);

/// 通过USB写入数据,需要初始化主设备USB接口
/// \param wdata 需要写入的数据
/// \param ilen 写入数据的长度 usb2.0(0:512] usb3.0(0:1024]
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_send_custom_data(const char *wdata,
                                                                    int ilen);

/// 读取USB数据,需要初始化主设备USB接口
/// \param ilen 需要读取的数据长度 [min:32,max：usb2.0:512 usb3.0:1024]
/// \return 读取到的数据
CARINA_A1088_VITURE_EXPORT char *carina_a1088_viture_read_custom_data(int ilen);

/// 切换显示模式
/// \param imode 0：2D 1:3D
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_switch_display_mode(const unsigned char imode);

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_reset_pose();

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT char *carina_a1088_viture_get_sdk_version();

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT char *carina_a1088_viture_get_firmware_version();

///
/// \return 0:success other:fail
CARINA_A1088_VITURE_EXPORT int carina_a1088_viture_set_low_power_mode(const bool &b_low_power);

#ifdef __cplusplus
}
#endif
#endif // CARINA_VIO_CARINA_A1088_VITURE_H
