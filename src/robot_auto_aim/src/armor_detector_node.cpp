#include "robot_auto_aim/armor_detector_node.hpp"

#include <algorithm>
#include <filesystem>
#include <functional>
#include <memory>
#include <numeric>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "robot_utils/url_resolver.hpp"

namespace robot_auto_aim {

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("armor_detector", options)
{
    RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode constructed.");
}

ArmorDetectorNode::~ArmorDetectorNode() = default;

CallbackReturn ArmorDetectorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(this->get_logger(), "Configuring ArmorDetectorNode...");

    detector_ = initDetector();
    use_ba_ = this->declare_parameter("use_ba", true);

    armors_pub_ = this->create_publisher<robot_interfaces::msg::Armors>("armor_detector/armors", rclcpp::SensorDataQoS());

    odom_frame_ = this->declare_parameter("target_frame", "odom");
    imu_to_camera_ = Eigen::Matrix3d::Identity();

    // Visualization
    armor_marker_.ns = "armors";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.y = 0.15;
    armor_marker_.scale.z = 0.12;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_detector/marker", 10);

    debug_ = this->declare_parameter("debug", true);
    if (debug_) {
        createDebugPublishers();
    }

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmorDetectorNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(this->get_logger(), "Activating ArmorDetectorNode...");

    armors_pub_->on_activate();
    marker_pub_->on_activate();
    if (debug_) {
        lights_data_pub_->on_activate();
        armors_data_pub_->on_activate();
        binary_img_pub_->on_activate();
        number_img_pub_->on_activate();
        result_img_pub_->on_activate();
    }

    // Subscribe to camera info
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/image_raw_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
            cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            armor_pose_estimator_ = std::make_unique<robot_auto_aim::ArmorPoseEstimator>(cam_info_);
            armor_pose_estimator_->enableBA(use_ba_);
            cam_info_sub_.reset();
        });

    // Subscribing to image only when active, to save CPU/Network when inactive
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmorDetectorNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(this->get_logger(), "Deactivating ArmorDetectorNode...");

    armors_pub_->on_deactivate();
    marker_pub_->on_deactivate();
    if (debug_) {
        lights_data_pub_->on_deactivate();
        armors_data_pub_->on_deactivate();
        binary_img_pub_->on_deactivate();
        number_img_pub_->on_deactivate();
        result_img_pub_->on_deactivate();
    }

    img_sub_.reset(); // Unsubscribe when inactive

    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmorDetectorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up ArmorDetectorNode...");
    
    detector_.reset();
    armor_pose_estimator_.reset();
    armors_pub_.reset();
    marker_pub_.reset();
    tf2_buffer_.reset();
    tf2_listener_.reset();

    if (debug_) {
        destroyDebugPublishers();
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmorDetectorNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(this->get_logger(), "Shutting down ArmorDetectorNode...");
    return CallbackReturn::SUCCESS;
}

void ArmorDetectorNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    try {
        rclcpp::Time target_time = img_msg->header.stamp;
        auto odom_to_gimbal = tf2_buffer_->lookupTransform(
            odom_frame_, img_msg->header.frame_id, target_time,
            rclcpp::Duration::from_seconds(0.01));
        auto msg_q = odom_to_gimbal.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3 tf2_matrix(tf_q);
        imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
                          tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
                          tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Something wrong when lookUpTransform: %s", ex.what());
        return;
    }

    auto armors = detectArmors(img_msg);

    armors_msg_.header = img_msg->header;
    armors_msg_.armors.clear();

    if (armor_pose_estimator_ != nullptr) {
        armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "ArmorPoseEstimator not initialized yet!");
    }

    if (debug_) {
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 100;
        armor_marker_.header = text_marker_.header = armors_msg_.header;
        
        for (const auto & armor : armors_msg_.armors) {
            armor_marker_.pose = armor.pose;
            armor_marker_.id++;
            text_marker_.pose.position = armor.pose.position;
            text_marker_.id++;
            text_marker_.pose.position.y -= 0.1;
            text_marker_.text = armor.number;
            marker_array_.markers.emplace_back(armor_marker_);
            marker_array_.markers.emplace_back(text_marker_);
        }
        marker_pub_->publish(marker_array_);
    }

    armors_pub_->publish(armors_msg_);
}

std::unique_ptr<robot_auto_aim::Detector> ArmorDetectorNode::initDetector()
{
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int binary_thres = this->declare_parameter("binary_thres", 160, param_desc);

    robot_auto_aim::Detector::LightParams l_params = {
        .min_ratio = this->declare_parameter("light.min_ratio", 0.08),
        .max_ratio = this->declare_parameter("light.max_ratio", 0.4),
        .max_angle = this->declare_parameter("light.max_angle", 40.0),
        .color_diff_thresh = static_cast<int>(this->declare_parameter("light.color_diff_thresh", 25))
    };

    robot_auto_aim::Detector::ArmorParams a_params = {
        .min_light_ratio = this->declare_parameter("armor.min_light_ratio", 0.6),
        .min_small_center_distance = this->declare_parameter("armor.min_small_center_distance", 0.8),
        .max_small_center_distance = this->declare_parameter("armor.max_small_center_distance", 3.2),
        .min_large_center_distance = this->declare_parameter("armor.min_large_center_distance", 3.2),
        .max_large_center_distance = this->declare_parameter("armor.max_large_center_distance", 5.0),
        .max_angle = this->declare_parameter("armor.max_angle", 35.0)
    };

    std::string enemy_color_ch = this->declare_parameter("enemy_color", "red");
    robot_utils::EnemyColor enemy_color = (enemy_color_ch == "red") ? robot_utils::EnemyColor::RED : robot_utils::EnemyColor::BLUE;
    
    auto detector = std::make_unique<robot_auto_aim::Detector>(binary_thres, enemy_color, l_params, a_params);

    namespace fs = std::filesystem;
    fs::path model_path = robot_utils::URLResolver::getResolvedPath("package://robot_auto_aim/model/lenet.onnx");
    fs::path label_path = robot_utils::URLResolver::getResolvedPath("package://robot_auto_aim/model/label.txt");
    
    if (!fs::exists(model_path) || !fs::exists(label_path)) {
        RCLCPP_ERROR(this->get_logger(), "ONNX model or label file not found at: %s", model_path.string().c_str());
    }

    double threshold = this->declare_parameter("classifier_threshold", 0.7);
    std::vector<std::string> ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
    detector->classifier = std::make_unique<robot_auto_aim::NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    bool use_pca = this->declare_parameter("use_pca", true);
    if (use_pca) {
        detector->corner_corrector = std::make_unique<robot_auto_aim::LightCornerCorrector>();
    }

    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArmorDetectorNode::onSetParameters, this, std::placeholders::_1));

    return detector;
}

std::vector<robot_auto_aim::Armor> ArmorDetectorNode::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    auto armors = detector_->detect(img);

    auto final_time = this->now();
    auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;

    if (debug_) {
        binary_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

        std::sort(detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
                  [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
        std::sort(detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
                  [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

        lights_data_pub_->publish(detector_->debug_lights);
        armors_data_pub_->publish(detector_->debug_armors);

        if (!armors.empty()) {
            auto all_num_img = detector_->getAllNumbersImage();
            number_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
        }

        detector_->drawResults(img);

        cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
        std::stringstream latency_ss;
        latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
        cv::putText(img, latency_ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        
        result_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    }

    return armors;
}

rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::onSetParameters(std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
        if (param.get_name() == "binary_thres") detector_->binary_thres = param.as_int();
        else if (param.get_name() == "classifier_threshold") detector_->classifier->threshold = param.as_double();
        else if (param.get_name() == "light.min_ratio") detector_->light_params.min_ratio = param.as_double();
        else if (param.get_name() == "light.max_ratio") detector_->light_params.max_ratio = param.as_double();
        else if (param.get_name() == "light.max_angle") detector_->light_params.max_angle = param.as_double();
        else if (param.get_name() == "light.color_diff_thresh") detector_->light_params.color_diff_thresh = param.as_int();
        else if (param.get_name() == "armor.min_light_ratio") detector_->armor_params.min_light_ratio = param.as_double();
        else if (param.get_name() == "armor.min_small_center_distance") detector_->armor_params.min_small_center_distance = param.as_double();
        else if (param.get_name() == "armor.max_small_center_distance") detector_->armor_params.max_small_center_distance = param.as_double();
        else if (param.get_name() == "armor.min_large_center_distance") detector_->armor_params.min_large_center_distance = param.as_double();
        else if (param.get_name() == "armor.max_large_center_distance") detector_->armor_params.max_large_center_distance = param.as_double();
        else if (param.get_name() == "armor.max_angle") detector_->armor_params.max_angle = param.as_double();
    }
    return result;
}

void ArmorDetectorNode::createDebugPublishers()
{
    lights_data_pub_ = this->create_publisher<robot_interfaces::msg::DebugLights>("armor_detector/debug_lights", 10);
    armors_data_pub_ = this->create_publisher<robot_interfaces::msg::DebugArmors>("armor_detector/debug_armors", 10);

    this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
    this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
    
    binary_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("armor_detector/binary_img", 10);
    number_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("armor_detector/number_img", 10);
    result_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("armor_detector/result_img", 10);
}

void ArmorDetectorNode::destroyDebugPublishers()
{
    lights_data_pub_.reset();
    armors_data_pub_.reset();
    binary_img_pub_.reset();
    number_img_pub_.reset();
    result_img_pub_.reset();
}

}  // namespace robot_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_auto_aim::ArmorDetectorNode)
