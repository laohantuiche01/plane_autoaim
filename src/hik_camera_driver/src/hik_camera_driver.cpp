//
// Created by mijiao on 24-4-15.
//

#include "hik_camera_driver.h"

std::map<std::string, const char *> HikCameraDriver::realName = {
    {"exposure_time", "ExposureTime"},
    {"gain", "Gain"},
    {"frame_rate", "AcquisitionFrameRate"},
    {"width", "Width"},
    {"height", "Height"},
    {"offset_x", "OffsetX"},
    {"offset_y", "OffsetY"},
    {"bit_depth", "ADCBitDepth"}
};

HikCameraDriver::HikCameraDriver() : HikCameraDriver(rclcpp::NodeOptions()) {
}

HikCameraDriver::HikCameraDriver(const rclcpp::NodeOptions &options) : Node("camera_node", options) {
    using namespace std::chrono_literals;
    declare_parameter("exposure_time", 1500.0);
    declare_parameter("gain", 15.0);
    declare_parameter("frame_rate", 250.0);
    declare_parameter("width", 1440);
    declare_parameter("height", 520);
    declare_parameter("offset_x", 0);
    declare_parameter("offset_y", 280);
    declare_parameter("bit_depth", "Bits_8");
    declare_parameter("k", std::vector<double>({
                          4637.68, 0, 720,
                          0, 4637.68, 540,
                          0, 0, 1
                      }));
    declare_parameter("d", std::vector<double>({0, 0, 0, 0, 0}));

    get_parameter("k", k);
    get_parameter("d", d);
    get_parameter("frame_rate", frame_rate_);
    get_parameter("width", width_);
    get_parameter("height", height_);
    get_parameter("offset_x", offset_x_);
    get_parameter("offset_y", offset_y_);

    imagePublisher = create_publisher<RosImage>("image_raw", 10);
    cameraInfoPublisher = create_publisher<CameraInfo>("camera_info", 1);
    timer = create_wall_timer(std::chrono::duration<double>(1.0 / frame_rate_),
                              std::bind(&HikCameraDriver::cameraCallback, this));
    onSetParametersCallbackHandle = add_on_set_parameters_callback(
        std::bind(&HikCameraDriver::setParams, this, std::placeholders::_1));
    enumerateAndSelectDevice();
    openDevice();
    setTriggerModeAndPixelFormat();
    startGrabbing();
    setParams(get_parameters(
        {"exposure_time", "gain", "frame_rate", "width", "height", "offset_x", "offset_y", "bit_depth"}));
}

void HikCameraDriver::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
    if (nullptr == pstMVDevInfo) {
        return;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
        unsigned int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        unsigned int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        unsigned int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        unsigned int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        RCLCPP_INFO(get_logger(), "UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        RCLCPP_INFO(get_logger(), "CurrentIp: %d.%d.%d.%d", nIp1, nIp2, nIp3, nIp4);
    } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
        RCLCPP_INFO(get_logger(), "UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        RCLCPP_INFO(get_logger(), "Serial Number: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        RCLCPP_INFO(get_logger(), "Device Number: %d", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    } else {
        RCLCPP_WARN(get_logger(), "Unsupported camera type!");
    }
}

void HikCameraDriver::enumerateAndSelectDevice() {
    MV_CC_DEVICE_INFO_LIST deviceInfoList{};
    CameraErrorCode errorCode = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceInfoList);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Enumerating devices failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }
    unsigned int index = 0;
    if (deviceInfoList.nDeviceNum > 0) {
        for (unsigned int i = 0; i < deviceInfoList.nDeviceNum; i++) {
            RCLCPP_INFO(get_logger(), "[device %d]:", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = deviceInfoList.pDeviceInfo[i];
            PrintDeviceInfo(pDeviceInfo);
            if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
                std::string target_camera_name((char *) pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
                if (target_camera_name == get_namespace()) {
                    index = i;
                }
            } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
                std::string target_camera_name((char *) pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
                if (target_camera_name == get_namespace()) {
                    index = i;
                }
            }
        }
    } else {
        RCLCPP_FATAL(get_logger(), "No device found!");
        exit(0);
    }
    errorCode = MV_CC_CreateHandle(&handle, deviceInfoList.pDeviceInfo[index]);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Creating handle failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }
}

void HikCameraDriver::openDevice() {
    CameraErrorCode errorCode = MV_CC_OpenDevice(handle);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Opening camera failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }
}

void HikCameraDriver::setTriggerModeAndPixelFormat() {
    CameraErrorCode errorCode = MV_CC_SetTriggerMode(handle, MV_TRIGGER_MODE_OFF);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Setting trigger mode failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }
    errorCode = MV_CC_SetPixelFormat(handle, MvGvspPixelType::PixelType_Gvsp_BayerRG8);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Setting Pixel format failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }
}

void HikCameraDriver::startGrabbing() {
    CameraErrorCode errorCode = MV_CC_StartGrabbing(handle);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Starting Grabbing failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }
}

void HikCameraDriver::cameraCallback() {
    static rclcpp::Time start = now();
    auto lastStart = start;
    start = now();
    RosImage::UniquePtr image = std::make_unique<RosImage>();
    MV_FRAME_OUT frameOut{};
    auto t1 = now();
    CameraErrorCode errorCode = MV_CC_GetImageBuffer(handle, &frameOut, 1000);
    auto t2 = now();
    RCLCPP_DEBUG(get_logger(), "Frame delta time: %f", (t2 - t1).seconds());
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Getting image buffer failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }

    cv::Mat bgrImage;
    cv::Mat bayerImage(frameOut.stFrameInfo.nHeight, frameOut.stFrameInfo.nWidth, CV_8UC1, frameOut.pBufAddr);
    cv::cvtColor(bayerImage, bgrImage, cv::COLOR_BayerRG2RGB);
    image->data = std::vector<unsigned char>(bgrImage.data, bgrImage.data +
                                                            frameOut.stFrameInfo.nWidth *
                                                            frameOut.stFrameInfo.nHeight * 3);

    image->height = frameOut.stFrameInfo.nHeight;
    image->width = frameOut.stFrameInfo.nWidth;
    image->step = frameOut.stFrameInfo.nWidth * 3;
    image->encoding = "bgr8";
    image->header.stamp = t2;
    image->header.frame_id = "camera_optical_frame";
    imagePublisher->publish(std::move(image));
    errorCode = MV_CC_FreeImageBuffer(handle, &frameOut);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Getting image buffer failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }

    CameraInfo cameraInfo;
    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.d = d;
    std::array<double, 9> roiK{};
    memcpy(roiK.data(), k.data(), 9 * sizeof(double));
    roiK[2] -= (double) offset_x_;
    roiK[5] -= (double) offset_y_;
    cameraInfo.k = roiK;
    cameraInfo.roi.x_offset = offset_x_;
    cameraInfo.roi.y_offset = offset_y_;
    cameraInfo.roi.height = height_;
    cameraInfo.roi.width = width_;
    cameraInfoPublisher->publish(cameraInfo);

    static struct Count {
        uint8_t i: 5;
    } count;
    static std::array<double, 32> recentDurations = {};
    recentDurations[count.i++] = (start - lastStart).seconds();
    double fps = (double) recentDurations.size() /
                 std::reduce(recentDurations.begin(), recentDurations.end(), 0.0);
    RCLCPP_DEBUG(get_logger(), "FPS: %f", fps);
}

rcl_interfaces::msg::SetParametersResult HikCameraDriver::setParams(const std::vector<rclcpp::Parameter> &parameters) {
    stopGrabbing();
    bool success = true;
    for (auto &parameter: parameters) {
        auto name = parameter.get_name();

        if (name == "k") {
            k = parameter.as_double_array();
            continue;
        } else if (name == "d") {
            d = parameter.as_double_array();
            continue;
        } else if (name == "width") {
            width_ = parameter.as_int();
        } else if (name == "height") {
            height_ = parameter.as_int();
        } else if (name == "offset_x") {
            offset_x_ = parameter.as_int();
        } else if (name == "offset_y") {
            offset_y_ = parameter.as_int();
        } else if (name == "frame_rate") {
            frame_rate_ = parameter.as_double();
            timer->cancel();
            timer = create_wall_timer(std::chrono::duration<double>(1.0 / frame_rate_),
                                      std::bind(&HikCameraDriver::cameraCallback, this));
        }

        if (realName.count(name)) {
            switch (parameter.get_type()) {
                case rclcpp::PARAMETER_INTEGER:
                    success &= setCameraValue(realName[name], parameter.as_int());
                    break;
                case rclcpp::PARAMETER_DOUBLE:
                    success &= setCameraValue(realName[name], (float) parameter.as_double());
                    break;
                case rclcpp::PARAMETER_STRING:
                    success &= setCameraValue(realName[name], parameter.as_string().c_str());
                    break;
                default:
                    break;
            }
        }
    }
    startGrabbing();
    return rcl_interfaces::msg::SetParametersResult().set__successful(success);
}

void HikCameraDriver::stopGrabbing() {
    CameraErrorCode errorCode = MV_CC_StopGrabbing(handle);
    if (errorCode != MV_OK) {
        RCLCPP_FATAL(get_logger(), "Stopping Grabbing failed! nRet [0x%x]", errorCode);
        exit(errorCode);
    }
}

bool HikCameraDriver::setCameraValue(const char *strKey, int64_t value) {
    CameraErrorCode errorCode;
    MVCC_INTVALUE_EX valueAttribute;
    errorCode = MV_CC_GetIntValueEx(handle, strKey, &valueAttribute);
    if (errorCode != MV_OK) {
        RCLCPP_WARN(get_logger(), "Getting %s failed! nRet [0x%x]", strKey, errorCode);
        return false;
    }

    if (valueAttribute.nCurValue == value) {
        RCLCPP_INFO(get_logger(), "%s is already %ld.", strKey, value);
        return true;
    }

    if (value > valueAttribute.nMax) {
        RCLCPP_WARN(get_logger(),
                    "New %s: %ld is too big! Set it to the max value: %ld", strKey, value, valueAttribute.nMax);
        value = valueAttribute.nMax;
    }

    if (value < valueAttribute.nMin) {
        RCLCPP_WARN(get_logger(),
                    "New %s: %ld is too small! Set it to the min value: %ld", strKey, value, valueAttribute.nMin);
        value = valueAttribute.nMin;
    }

    if ((value - valueAttribute.nMin) % valueAttribute.nInc) {
        value = (value - valueAttribute.nMin) / valueAttribute.nInc + valueAttribute.nMin;
        RCLCPP_WARN(get_logger(), "New %s is illegal! Set it to the closest value: %ld", strKey, value);
    }

    errorCode = MV_CC_SetIntValue(handle, strKey, value);
    if (errorCode != MV_OK) {
        RCLCPP_WARN(get_logger(), "Setting %s failed! nRet [0x%x]", strKey, errorCode);
        return false;
    }

    RCLCPP_INFO(get_logger(), "Setting %s to %ld success!", strKey, value);
    return true;
}

bool HikCameraDriver::setCameraValue(const char *strKey, float value) {
    CameraErrorCode errorCode;
    MVCC_FLOATVALUE valueAttribute;
    errorCode = MV_CC_GetFloatValue(handle, strKey, &valueAttribute);
    if (errorCode != MV_OK) {
        RCLCPP_WARN(get_logger(), "Getting %s failed! nRet [0x%x]", strKey, errorCode);
        return false;
    }

    if (valueAttribute.fCurValue == value) {
        RCLCPP_INFO(get_logger(), "%s is already %f.", strKey, value);
        return true;
    }

    if (value > valueAttribute.fMax) {
        RCLCPP_WARN(get_logger(),
                    "New %s: %f is too big! Set it to the max value: %f", strKey, value, valueAttribute.fMax);
        value = valueAttribute.fMax;
    }

    if (value < valueAttribute.fMin) {
        RCLCPP_WARN(get_logger(),
                    "New %s: %f is too small! Set it to the min value: %f", strKey, value, valueAttribute.fMin);
        value = valueAttribute.fMin;
    }

    errorCode = MV_CC_SetFloatValue(handle, strKey, value);
    if (errorCode != MV_OK) {
        RCLCPP_WARN(get_logger(), "Setting %s failed! nRet [0x%x]", strKey, errorCode);
        return false;
    }

    RCLCPP_INFO(get_logger(), "Setting %s to %f success!", strKey, value);
    return true;
}

bool HikCameraDriver::setCameraValue(const char *strKey, const char *value) {
    CameraErrorCode errorCode;
    MVCC_ENUMVALUE valueAttribute;
    errorCode = MV_CC_GetEnumValue(handle, strKey, &valueAttribute);
    if (errorCode != MV_OK) {
        RCLCPP_WARN(get_logger(), "Getting %s failed! nRet [0x%x]", strKey, errorCode);
        return false;
    }

    bool found = false;
    for (int i = 0; i < (int) valueAttribute.nSupportedNum; i++) {
        MVCC_ENUMENTRY enumEntry;
        enumEntry.nValue = valueAttribute.nSupportValue[i];
        errorCode = MV_CC_GetEnumEntrySymbolic(handle, strKey, &enumEntry);
        if (errorCode != MV_OK) {
            RCLCPP_WARN(get_logger(), "Getting %s failed! nRet [0x%x]", strKey, errorCode);
            return false;
        }
        if (strcmp(enumEntry.chSymbolic, value) == 0) {
            found = true;
            if (valueAttribute.nSupportValue[i] == valueAttribute.nCurValue) {
                RCLCPP_INFO(get_logger(), "%s is already %s.", strKey, value);
                return true;
            }
        }
    }

    if (!found) {
        RCLCPP_WARN(get_logger(), "Enum %s doesn't have value called %s.", strKey, value);
        return false;
    }

    errorCode = MV_CC_SetEnumValueByString(handle, strKey, value);
    if (errorCode != MV_OK) {
        RCLCPP_WARN(get_logger(), "Setting %s failed! nRet [0x%x]", strKey, errorCode);
        return false;
    }

    RCLCPP_INFO(get_logger(), "Setting %s to %s success!", strKey, value);
    return true;
}


