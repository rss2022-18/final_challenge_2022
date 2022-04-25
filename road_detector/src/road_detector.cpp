#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <algorithm>
const int low_threshold = 100;
const int high_threshold = 200;
const int kernel_size = 3;
const char* window_name = "Road Detector";
const double METERS_PER_INCH = 0.0254;
const uint32_t shape = visualization_msgs::Marker::CUBE;
const int PTS_IMAGE_PLANE[4][2] = [[253, 203],
                   [471, 201],
                   [281, 178],
                   [441, 245]];
const double PTS_GROUND_PLANE[4][2] = [[-12.75** METERS_PER_INCH, 39.3701** METERS_PER_INCH],
                    [12.75** METERS_PER_INCH, 39.3701** METERS_PER_INCH],
                    [-12.75** METERS_PER_INCH, 50.3701** METERS_PER_INCH],
                    [4.25** METERS_PER_INCH, 23.0** METERS_PER_INCH]];

class RoadDetector{
    protected:
    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    ros::Publisher img_pub_;
    ros::Publisher box_pub_;
    ros::Publisher marker_pub_; 

    private:
    cv::Mat img_; 
    cv::Mat debug_img_;
    cv::Mat detected_edges_;
    cv::Mat homography_matrix_;
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
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/road_detector/points", 1);
            homograph_matrix = cv::findHomography(cv::Mat(4, 2, CV_32FC1, PTS_IMAGE_PLANE), cv::Mat(4, 2, CV_32FC1, PTS_GROUND_PLANE));
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
            cv::Mat thresh_white;
            cv::inRange(img_, cv::Scalar(0, 0, 200), cv::Scalar(255, 255, 255), thresh_white);
            cv::Canny(thresh_white, detected_edges_, low_threshold_, high_threshold_, kernel_size_);

            // Crop the image(remove top half)
            cv::Rect left_roi(0, detected_edges_.rows/2, detected_edges_.cols/2, detected_edges_.rows/2);
            cv::Rect right_roi(detected_edges_.cols/2, detected_edges_.rows/2, detected_edges_.cols/2, detected_edges_.rows/2);
            cv::Mat left_img = detected_edges_(left_roi);
            cv::Mat right_img = detected_edges_(right_roi);

            std::vector<cv::Vec4i> left_lines;
            std::vector<cv::Vec4i> right_lines; 
            cv::HoughLinesP(left_img, left_lines, 1, CV_PI/180, 50, 50, 10);
            cv::HoughLinesP(right_img, right_lines, 1, CV_PI/180, 50, 50, 10);
 
            double min_dist = 100000;
            for(size_t i = 0; i < left_lines.size(); i++){
                cv::Vec4i l = left_lines[i];
                double dist = RoadDetector::getDist(l);
                if(dist < min_dist){
                    min_dist = dist;
                    left_line_ = l;
                }
                cv::line(debug_img_, cv::Point(l[0], l[1] + left_roi.y), cv::Point(l[2], l[3] + left_roi.y), cv::Scalar(0,0,255), 3, cv::LINE_AA);
            }
            min_dist = 100000;
            for(size_t i = 0; i < right_lines.size(); i++){
                cv::Vec4i l = right_lines[i];
                double dist = RoadDetector::getDist(l);
                if(dist < min_dist){
                    min_dist = dist;
                    right_line_ = l;
                }
                cv::line(debug_img_, cv::Point(l[0] + right_roi.x, l[1] + right_roi.y), cv::Point(l[2] + right_roi.x, l[3] + right_roi.y), cv::Scalar(0,0,255), 3, cv::LINE_AA);
            }
            int min_y, max_y;
            min_y = std::min(std::min(left_line_[1], left_line_[3]),std::min(right_line_[1], right_line_[3]));
            max_y = std::max(std::max(left_line_[1], left_line_[3]), std::max(right_line_[1], right_line_[3]));
            cv::Vec4i mid_line_ = cv::Vec4i((left_line_[0] + (right_line_[2] + right_roi.x))/2, min_y, ((right_line_[2] + right_roi.x) + left_line_[0])/2, max_y);
            cv::line(debug_img_, cv::Point(left_line_[0], left_line_[1] + left_roi.y), cv::Point(left_line_[2], left_line_[3] + left_roi.y), cv::Scalar(0,255,0), 3, cv::LINE_AA);
            cv::line(debug_img_, cv::Point(right_line_[0] + right_roi.x , right_line_[1] + right_roi.y), cv::Point(right_line_[2] + right_roi.x, right_line_[3] + right_roi.y), cv::Scalar(0,255,0), 3, cv::LINE_AA);
            cv::line(debug_img_, cv::Point(mid_line_[0], mid_line_[1] + left_roi.y), cv::Point(mid_line_[2], mid_line_[3] + left_roi.y), cv::Scalar(255,0,0), 3, cv::LINE_AA);

            img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img_).toImageMsg());
        }
    double getDist(cv::Vec4i line){ 
        double m = (line[3] - line[1])/(line[2] - line[0]);
        double b = line[1] - line[0]*m;
        return abs(m*detected_edges_.cols + b - detected_edges_.rows/2)/sqrt(m*m + 1);
    }
    void transformUvToXy(int u, int v, double& x, double& y){
        cv::Vec3i homogeneous_point = cv::Vec3i(u, v, 1);
        cv::Vec3d xy_point =  homography_matrix.dot(homogeneous_point);
        double scaling_factor = 1/xy_point[2];
        homoogeneous_xy = xy_point * scaling_factor;
        *x = homoogeneous_xy[0];
        *y = homoogeneous_xy[1];
    }

    void visualizePoint(int u, int v){
        double x,y;
        RoadDetector::transformUvToXy(u,v,&x, &y);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/zed_camera_center";
        marker.header.stamp = ros::Time::now();
        marker.ns = "road_detector";
        marker.id = 0;
        marker.type = marker_shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x; 
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.lifetime = ros::Duration();
        marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "road_detector");
    ros::NodeHandle nh;
    RoadDetector rd(nh);
    ros::spin();
    return 0;
}