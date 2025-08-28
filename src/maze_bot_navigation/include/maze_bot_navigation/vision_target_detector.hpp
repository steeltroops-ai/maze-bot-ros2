#ifndef VISION_TARGET_DETECTOR_HPP
#define VISION_TARGET_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.hpp>

namespace maze_bot_navigation
{

struct TargetDetection
{
    cv::Point2f center;
    double confidence;
    double distance;
    double angle;
    bool valid;
};

class VisionTargetDetector : public rclcpp::Node
{
public:
    VisionTargetDetector();
    ~VisionTargetDetector();

    // Main detection function
    TargetDetection detectTarget(const cv::Mat& image);
    
    // Get latest detection result
    TargetDetection getLatestDetection() const { return latest_detection_; }
    
    // Check if target is currently visible
    bool isTargetVisible() const { return latest_detection_.valid; }
    
    // Get target position in robot frame
    geometry_msgs::msg::Point getTargetPosition() const;

private:
    // ROS2 subscribers and publishers
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    
    // Camera parameters
    double fx_, fy_, cx_, cy_;  // Camera intrinsics
    bool camera_info_received_;
    
    // Target detection parameters
    struct DetectionParams
    {
        // Color-based detection (HSV ranges)
        cv::Scalar target_color_lower;
        cv::Scalar target_color_upper;
        
        // Shape-based detection
        double min_contour_area;
        double max_contour_area;
        double min_circularity;
        double min_convexity;
        
        // Distance estimation
        double known_target_size;  // Real-world size of target (meters)
        double max_detection_distance;
        
        // Confidence thresholds
        double min_confidence;
        double temporal_consistency_threshold;
    } params_;
    
    // Detection state
    TargetDetection latest_detection_;
    std::vector<TargetDetection> detection_history_;
    rclcpp::Time last_detection_time_;
    
    // Callback functions
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    
    // Detection algorithms
    TargetDetection detectByColor(const cv::Mat& image);
    TargetDetection detectByShape(const cv::Mat& image);
    TargetDetection detectByTemplate(const cv::Mat& image);
    TargetDetection detectByFeatures(const cv::Mat& image);
    
    // Utility functions
    double estimateDistance(const cv::Rect& bounding_box);
    double calculateAngle(const cv::Point2f& center, const cv::Size& image_size);
    double calculateConfidence(const cv::Mat& roi, const cv::Rect& bbox);
    bool validateDetection(const TargetDetection& detection);
    void updateDetectionHistory(const TargetDetection& detection);
    TargetDetection filterDetections();
    
    // Debug and visualization
    void publishDebugImage(const cv::Mat& image, const TargetDetection& detection);
    void drawDetection(cv::Mat& image, const TargetDetection& detection);
};

// Target recognition using machine learning (optional enhancement)
class MLTargetRecognizer
{
public:
    MLTargetRecognizer();
    ~MLTargetRecognizer();
    
    bool loadModel(const std::string& model_path);
    TargetDetection recognizeTarget(const cv::Mat& image);
    
private:
    cv::dnn::Net net_;
    bool model_loaded_;
    std::vector<std::string> class_names_;
    
    // Preprocessing and postprocessing
    cv::Mat preprocessImage(const cv::Mat& image);
    std::vector<TargetDetection> postprocessDetections(const cv::Mat& output);
};

// Multi-modal target detection combining different approaches
class MultiModalTargetDetector
{
public:
    MultiModalTargetDetector();
    ~MultiModalTargetDetector();
    
    TargetDetection detectTarget(const cv::Mat& image);
    void setDetectionMode(const std::string& mode);
    
private:
    std::unique_ptr<VisionTargetDetector> color_detector_;
    std::unique_ptr<MLTargetRecognizer> ml_recognizer_;
    
    std::string current_mode_;
    
    // Fusion algorithms
    TargetDetection fuseDetections(const std::vector<TargetDetection>& detections);
    double calculateFusionWeight(const TargetDetection& detection, const std::string& method);
};

} // namespace maze_bot_navigation

#endif // VISION_TARGET_DETECTOR_HPP
