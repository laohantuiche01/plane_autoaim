//
// Created by mijiao on 24-4-15.
//

#ifndef HIK_CAMERA_HIK_CAMERA_H
#define HIK_CAMERA_HIK_CAMERA_H

#include <MvCameraControl.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class HikCameraDriver : public rclcpp::Node {
private:
    using CameraErrorCode = uint8_t;
    using CameraHandle = void *;
    using RosImage = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    static std::map<std::string, const char *> realName;

    std::vector<double> k;
    std::vector<double> d;
    CameraHandle handle = nullptr;
    rclcpp::Publisher<RosImage>::SharedPtr imagePublisher;
    std::weak_ptr<std::remove_pointer<decltype(imagePublisher.get())>::type> capturedPublisher;
    rclcpp::Publisher<CameraInfo>::SharedPtr cameraInfoPublisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr onSetParametersCallbackHandle;

    void PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);

    void enumerateAndSelectDevice();

    void openDevice();

    void setTriggerModeAndPixelFormat();

    rcl_interfaces::msg::SetParametersResult setParams(const std::vector<rclcpp::Parameter> &parameters);

    void startGrabbing();

    void stopGrabbing();

    void cameraCallback();

    bool setCameraValue(const char *strKey, int64_t value);

    bool setCameraValue(const char *strKey, float value);

    bool setCameraValue(const char *strKey, const char *value);

public:
    HikCameraDriver();

    HikCameraDriver(const rclcpp::NodeOptions &options);
};


#endif //HIK_CAMERA_HIK_CAMERA_H
