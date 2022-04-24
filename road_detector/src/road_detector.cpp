#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

const int low_threshold = 100;
const int high_threshold = 200;
const int kernel_size = 3;
const char* window_name = "Road Detector";
class RoadDetector{
    protected:
    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    ros::Publisher img_pub_;
    ros::Publisher box_pub_;

    private:
    cv::Mat img_; 
    cv::Mat debug_img_;
    cv::Mat detected_edges_;
    int low_threshold_;
    int high_threshold_;
    int kernel_size_;
    cv::Vec4i left_line_;
    cv::Vec4i right_line_;

    public: 
        RoadDetector(ros::NodeHandle n): nh_(n), low_threshold_(200), high_threshold_(400), kernel_size_(3){
            img_sub_ = nh_.subscribe("/zed/zed_node/rgb/image_rect_color", 1, &RoadDetector::imgCallback, this);
            img_pub_ = nh_.advertise<sensor_msgs::Image>("/road_detector/debug_img", 1);
            box_pub_ = nh_.advertise<sensor_msgs::Image>("/road_detector/box", 1);
        }

        void imgCallback(const sensor_msgs::ImageConstPtr& msg){
            cv_bridge::CvImagePtr cv_ptr; 
            try{ 
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            img_ = cv_ptr->image; 
            debug_img_ = img_.clone();

            // Color Segmentation
            cv::cvtColor(img_, img_, cv::COLOR_BGR2HSV); 
            // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
            // cv::erode(img_, img_, element);
            // cv::dilate(img_, img_, element);
            cv::Mat thresh_white;
            cv::inRange(img_, cv::Scalar(0, 0, 200), cv::Scalar(255, 255, 255), thresh_white);

            // Canny Edge Detection
            // std::vector<std::vector<cv::Point> > contours;
            // cv::findContours(thresh_white, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::Canny(thresh_white, detected_edges_, low_threshold_, high_threshold_, kernel_size_);

            // Crop the image(remove top half)
            cv::Rect roi(0, detected_edges_.rows/2, detected_edges_.cols, detected_edges_.rows/2);
            cv::Mat cropped_img = detected_edges_(roi);

            //Detect Line segments using Hough Transform
            std::vector<cv::Vec4i> lines;
            cv::HoughLinesP(cropped_img, lines, 1, CV_PI/180, 50, 50, 10);

            // Write lines to debug Image and publish
            for(size_t i = 0; i < lines.size(); i++){
                cv::Vec4i l = lines[i];
                cv::line(debug_img_, cv::Point(l[0], l[1] + detected_edges_.rows/2), cv::Point(l[2], l[3] + detected_edges_.rows/2), cv::Scalar(0,0,255), 3, cv::LINE_AA);
            }
            img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img_).toImageMsg());
        }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "road_detector");
    ros::NodeHandle nh;
    RoadDetector rd(nh);
    ros::spin();
    return 0;
}