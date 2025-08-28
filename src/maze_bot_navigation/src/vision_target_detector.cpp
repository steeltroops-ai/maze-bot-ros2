#include "maze_bot_navigation/vision_target_detector.hpp"

namespace maze_bot_navigation
{

VisionTargetDetector::VisionTargetDetector()
    : Node("vision_target_detector"),
      it_(shared_from_this()),
      camera_info_received_(false)
{
    // Initialize detection parameters
    params_.target_color_lower = cv::Scalar(0, 100, 100);    // Red target (HSV)
    params_.target_color_upper = cv::Scalar(10, 255, 255);
    params_.min_contour_area = 500.0;
    params_.max_contour_area = 50000.0;
    params_.min_circularity = 0.3;
    params_.min_convexity = 0.8;
    params_.known_target_size = 0.2;  // 20cm target
    params_.max_detection_distance = 10.0;
    params_.min_confidence = 0.5;
    params_.temporal_consistency_threshold = 0.3;
    
    // Initialize detection state
    latest_detection_.valid = false;
    last_detection_time_ = this->now();
    
    // ROS2 subscribers and publishers
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
        std::bind(&VisionTargetDetector::imageCallback, this, std::placeholders::_1));
    
    debug_image_pub_ = it_.advertise("/vision/debug_image", 1);
    
    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/vision/target_position", 10);
    
    // Subscribe to camera info
    auto camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 1,
        std::bind(&VisionTargetDetector::cameraInfoCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Vision Target Detector initialized");
}

VisionTargetDetector::~VisionTargetDetector()
{
}

void VisionTargetDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Camera info not received yet, skipping detection");
        return;
    }
    
    try {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
        // Perform target detection
        TargetDetection detection = detectTarget(image);
        
        // Update detection history and filter
        updateDetectionHistory(detection);
        latest_detection_ = filterDetections();
        
        // Publish target position if valid
        if (latest_detection_.valid) {
            geometry_msgs::msg::PointStamped target_msg;
            target_msg.header.stamp = msg->header.stamp;
            target_msg.header.frame_id = "base_link";
            target_msg.point = getTargetPosition();
            target_pub_->publish(target_msg);
        }
        
        // Publish debug image
        publishDebugImage(image, latest_detection_);
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void VisionTargetDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
    if (!camera_info_received_) {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
            fx_, fy_, cx_, cy_);
    }
}

TargetDetection VisionTargetDetector::detectTarget(const cv::Mat& image)
{
    // Try multiple detection methods and fuse results
    std::vector<TargetDetection> detections;
    
    // Color-based detection
    TargetDetection color_detection = detectByColor(image);
    if (color_detection.valid) {
        detections.push_back(color_detection);
    }
    
    // Shape-based detection
    TargetDetection shape_detection = detectByShape(image);
    if (shape_detection.valid) {
        detections.push_back(shape_detection);
    }
    
    // Feature-based detection
    TargetDetection feature_detection = detectByFeatures(image);
    if (feature_detection.valid) {
        detections.push_back(feature_detection);
    }
    
    // Fuse detections if multiple methods found targets
    if (detections.empty()) {
        TargetDetection invalid_detection;
        invalid_detection.valid = false;
        invalid_detection.confidence = 0.0;
        invalid_detection.distance = 0.0;
        invalid_detection.angle = 0.0;
        invalid_detection.center = cv::Point2f(0, 0);
        return invalid_detection;
    }
    
    // Return the detection with highest confidence
    TargetDetection best_detection = detections[0];
    for (const auto& detection : detections) {
        if (detection.confidence > best_detection.confidence) {
            best_detection = detection;
        }
    }
    
    return best_detection;
}

TargetDetection VisionTargetDetector::detectByColor(const cv::Mat& image)
{
    TargetDetection detection;
    detection.valid = false;
    detection.confidence = 0.0;
    detection.distance = 0.0;
    detection.angle = 0.0;
    detection.center = cv::Point2f(0, 0);
    
    // Convert to HSV for better color detection
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    
    // Create mask for target color
    cv::Mat mask;
    cv::inRange(hsv, params_.target_color_lower, params_.target_color_upper, mask);
    
    // Morphological operations to clean up mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
        return detection;
    }
    
    // Find the largest valid contour
    double max_area = 0;
    int best_contour_idx = -1;
    
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area >= params_.min_contour_area && area <= params_.max_contour_area && area > max_area) {
            max_area = area;
            best_contour_idx = static_cast<int>(i);
        }
    }
    
    if (best_contour_idx >= 0) {
        // Calculate bounding box and center
        cv::Rect bbox = cv::boundingRect(contours[best_contour_idx]);
        detection.center = cv::Point2f(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
        
        // Estimate distance and angle
        detection.distance = estimateDistance(bbox);
        detection.angle = calculateAngle(detection.center, image.size());
        
        // Calculate confidence based on color match and shape
        cv::Mat roi = image(bbox);
        detection.confidence = calculateConfidence(roi, bbox);
        
        detection.valid = validateDetection(detection);
    }
    
    return detection;
}

TargetDetection VisionTargetDetector::detectByShape(const cv::Mat& image)
{
    TargetDetection detection;
    detection.valid = false;
    
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // Apply Gaussian blur
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
    
    // Use HoughCircles to detect circular targets
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 16, 100, 30, 10, 200);
    
    if (!circles.empty()) {
        // Find the most confident circle detection
        cv::Vec3f best_circle = circles[0];
        double best_confidence = 0;
        
        for (const auto& circle : circles) {
            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            
            // Calculate confidence based on circle properties
            cv::Rect bbox(center.x - radius, center.y - radius, 2 * radius, 2 * radius);
            bbox &= cv::Rect(0, 0, image.cols, image.rows);  // Ensure bbox is within image
            
            if (bbox.area() > 0) {
                cv::Mat roi = image(bbox);
                double confidence = calculateConfidence(roi, bbox);
                
                if (confidence > best_confidence) {
                    best_confidence = confidence;
                    best_circle = circle;
                }
            }
        }
        
        if (best_confidence > params_.min_confidence) {
            detection.center = cv::Point2f(best_circle[0], best_circle[1]);
            int radius = cvRound(best_circle[2]);
            cv::Rect bbox(detection.center.x - radius, detection.center.y - radius, 2 * radius, 2 * radius);
            
            detection.distance = estimateDistance(bbox);
            detection.angle = calculateAngle(detection.center, image.size());
            detection.confidence = best_confidence;
            detection.valid = validateDetection(detection);
        }
    }
    
    return detection;
}

TargetDetection VisionTargetDetector::detectByFeatures(const cv::Mat& /* image */)
{
    TargetDetection detection;
    detection.valid = false;

    // This is a placeholder for feature-based detection
    // In a full implementation, you would use ORB, SIFT, or other feature detectors
    // to match against a template of the target

    return detection;
}

double VisionTargetDetector::estimateDistance(const cv::Rect& bounding_box)
{
    if (!camera_info_received_) {
        return -1.0;
    }
    
    // Estimate distance based on known target size and camera parameters
    double pixel_size = std::max(bounding_box.width, bounding_box.height);
    double distance = (params_.known_target_size * fx_) / pixel_size;
    
    return std::min(distance, params_.max_detection_distance);
}

double VisionTargetDetector::calculateAngle(const cv::Point2f& center, const cv::Size& /* image_size */)
{
    if (!camera_info_received_) {
        return 0.0;
    }
    
    // Calculate angle from camera center
    double pixel_offset = center.x - cx_;
    double angle = std::atan(pixel_offset / fx_);
    
    return angle;
}

double VisionTargetDetector::calculateConfidence(const cv::Mat& roi, const cv::Rect& bbox)
{
    // Calculate confidence based on multiple factors
    double confidence = 0.0;
    
    // Factor 1: Size appropriateness
    double area = bbox.area();
    if (area >= params_.min_contour_area && area <= params_.max_contour_area) {
        confidence += 0.3;
    }
    
    // Factor 2: Color consistency (simplified)
    cv::Scalar mean_color = cv::mean(roi);
    if (mean_color[2] > mean_color[1] && mean_color[2] > mean_color[0]) {  // Reddish
        confidence += 0.4;
    }
    
    // Factor 3: Shape regularity
    double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
    if (aspect_ratio > 0.7 && aspect_ratio < 1.3) {  // Roughly square
        confidence += 0.3;
    }
    
    return confidence;
}

bool VisionTargetDetector::validateDetection(const TargetDetection& detection)
{
    return detection.confidence >= params_.min_confidence &&
           detection.distance > 0 &&
           detection.distance <= params_.max_detection_distance;
}

void VisionTargetDetector::updateDetectionHistory(const TargetDetection& detection)
{
    detection_history_.push_back(detection);
    
    // Keep only recent detections (last 10)
    if (detection_history_.size() > 10) {
        detection_history_.erase(detection_history_.begin());
    }
}

TargetDetection VisionTargetDetector::filterDetections()
{
    if (detection_history_.empty()) {
        TargetDetection invalid;
        invalid.valid = false;
        return invalid;
    }
    
    // Simple temporal filtering - return most recent valid detection
    for (auto it = detection_history_.rbegin(); it != detection_history_.rend(); ++it) {
        if (it->valid) {
            return *it;
        }
    }
    
    TargetDetection invalid;
    invalid.valid = false;
    return invalid;
}

geometry_msgs::msg::Point VisionTargetDetector::getTargetPosition() const
{
    geometry_msgs::msg::Point point;
    
    if (latest_detection_.valid) {
        // Convert from camera frame to robot frame
        double distance = latest_detection_.distance;
        double angle = latest_detection_.angle;
        
        point.x = distance * std::cos(angle);
        point.y = distance * std::sin(angle);
        point.z = 0.0;
    }
    
    return point;
}

void VisionTargetDetector::publishDebugImage(const cv::Mat& image, const TargetDetection& detection)
{
    cv::Mat debug_image = image.clone();
    
    if (detection.valid) {
        drawDetection(debug_image, detection);
    }
    
    // Convert back to ROS message and publish
    sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg();
    debug_image_pub_.publish(debug_msg);
}

void VisionTargetDetector::drawDetection(cv::Mat& image, const TargetDetection& detection)
{
    // Draw center point
    cv::circle(image, detection.center, 5, cv::Scalar(0, 255, 0), -1);
    
    // Draw distance and angle text
    std::string info = "D: " + std::to_string(detection.distance) + "m, " +
                      "A: " + std::to_string(detection.angle * 180.0 / M_PI) + "deg, " +
                      "C: " + std::to_string(detection.confidence);
    
    cv::putText(image, info, cv::Point(detection.center.x + 10, detection.center.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
}

} // namespace maze_bot_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<maze_bot_navigation::VisionTargetDetector>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in vision target detector: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
