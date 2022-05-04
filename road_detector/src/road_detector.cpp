#include "cv_bridge/cv_bridge.h"
#include "opencv2/calib3d/calib3d.hpp" 
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "final_challenge_2022/ConeLocation.h"
#include <algorithm>
#include <stdio.h>
#include <cmath>

const int low_threshold = 100;
const int high_threshold = 200;
const int kernel_size = 3;
const double METERS_PER_INCH = 0.0254;
std::vector<cv::Point2f> PTS_IMAGE_PLANE = {cv::Point2f(253, 203),
                   cv::Point2f(471, 201),
                   cv::Point2f(281, 178),
                   cv::Point2f(441, 245)};
std::vector<cv::Point2f> PTS_GROUND_PLANE = {cv::Point2f(-12.75* METERS_PER_INCH, 39.3701* METERS_PER_INCH),
                    cv::Point2f(12.75* METERS_PER_INCH, 39.3701* METERS_PER_INCH),
                    cv::Point2f(-12.75* METERS_PER_INCH, 50.3701* METERS_PER_INCH),
                    cv::Point2f(4.25* METERS_PER_INCH, 23.0* METERS_PER_INCH)};

class RoadDetector{
    protected:
    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    ros::Publisher img_pub_;
    ros::Publisher box_pub_;
    ros::Publisher marker_pub_; 
    ros::Publisher point_pub_;
    ros::Publisher angle_pub_;

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
    cv::Vec4i mid_line_;
    visualization_msgs::Marker line_list_;
    public: 
        RoadDetector(ros::NodeHandle n): nh_(n), low_threshold_(200), high_threshold_(400), kernel_size_(3){
            img_sub_ = nh_.subscribe("/zed/zed_node/rgb/image_rect_color", 1, &RoadDetector::imgCallback, this);
            img_pub_ = nh_.advertise<sensor_msgs::Image>("/road_detector/debug_img", 1);
            box_pub_ = nh_.advertise<sensor_msgs::Image>("/road_detector/box", 1);
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/road_detector/visualizations", 1);
            point_pub_ = nh_.advertise<final_challenge_2022::ConeLocation>("/road_detector/next_point", 1);
            angle_pub_ = nh_.advertise<std_msgs::Float32>("/road_detector/angle",1);
            homography_matrix_ = cv::findHomography(PTS_IMAGE_PLANE, PTS_GROUND_PLANE);
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
            cv::HoughLinesP(left_img, left_lines, 1, CV_PI/180, 50, 100, 20);
            cv::HoughLinesP(right_img, right_lines, 1, CV_PI/180, 50, 100, 20);
 
            double min_dist = 100000;
            for(size_t i = 0; i < left_lines.size(); i++){
                cv::Vec4i l = left_lines[i];
                double m = (l[3] - l[1])/(l[2] - l[0] +.0001);
                double b = l[1] - l[0]*m;
                double dist = RoadDetector::getDist(l,m,b);
                double length = std::sqrt((l[2]- l[0])*(l[2]- l[0]) + (l[3]- l[1])*(l[3]- l[1]));
                bool cross = (l[0] < detected_edges_.cols/2) ^(l[2] < detected_edges_.cols/2);
                if(dist < min_dist &&  std::abs(m) > 0.25 && length >150){
                    min_dist = dist;
                    left_line_ = generate_lane(m,b);
                }
                cv::line(debug_img_, cv::Point(l[0], l[1] + left_roi.y), cv::Point(l[2], l[3] + left_roi.y), cv::Scalar(0,0,255), 3, cv::LINE_AA);
            }
            min_dist = 100000;
            for(size_t i = 0; i < right_lines.size(); i++){
                cv::Vec4i l = right_lines[i];
                double m = (l[3] - l[1])/(l[2] - l[0] +.0001);
                double b = l[1] - l[0]*m;
                double dist = RoadDetector::getDist(l,m,b);
                double length = std::sqrt((l[2]- l[0])*(l[2]- l[0]) + (l[3]- l[1])*(l[3]- l[1]));
                bool cross = (l[0] < detected_edges_.cols/2) ^ (l[2] < detected_edges_.cols/2);
                if(dist < min_dist && std::abs(m) > 0.25 && length >150){
                    min_dist = dist;
                    right_line_ = generate_lane(m ,b);
                }
                cv::line(debug_img_, cv::Point(l[0] + right_roi.x, l[1] + right_roi.y), cv::Point(l[2] + right_roi.x, l[3] + right_roi.y), cv::Scalar(0,0,255), 3, cv::LINE_AA);
            }
            int min_y, max_y;
            min_y = std::min(std::min(left_line_[1], left_line_[3]),std::min(right_line_[1], right_line_[3]));
            max_y = std::max(std::max(left_line_[1], left_line_[3]), std::max(right_line_[1], right_line_[3]));
            int x_offset = (std::max(left_line_[0],left_line_[2])+ std::min(right_line_[0], right_line_[2]) + right_roi.x)/2;
            // mid_line_ = cv::Vec4i((left_line_[0] + (right_line_[2] + right_roi.x))/2, min_y, ((right_line_[0] + right_roi.x) + left_line_[0])/2, max_y);
            mid_line_ = cv::Vec4i(x_offset, detected_edges_.rows/2,detected_edges_.cols/2,detected_edges_.rows);
            cv::line(debug_img_, cv::Point(left_line_[0], left_line_[1] + left_roi.y), cv::Point(left_line_[2], left_line_[3] + left_roi.y), cv::Scalar(0,255,0), 3, cv::LINE_AA);
            cv::line(debug_img_, cv::Point(right_line_[0] + right_roi.x , right_line_[1] + right_roi.y), cv::Point(right_line_[2] + right_roi.x, right_line_[3] + right_roi.y), cv::Scalar(0,255,0), 3, cv::LINE_AA);
            cv::line(debug_img_, cv::Point(mid_line_[0], mid_line_[1]), cv::Point(mid_line_[2], mid_line_[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
            // img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img_).toImageMsg());
            // final_challenge_2022::ConeLocation next_point;
            // double x,y;
            // RoadDetector::transformUvToXy(mid_line_[0], mid_line_[1], &x, &y);
            // next_point.x_pos = x;
            // next_point.y_pos = y;
            // point_pub_.publish(next_point);
            // Calculate arctan using x and y offset

            // Max deviation of 10 degrees
            double angle = std::max(std::atan(x_offset/(detected_edges_.rows/2)) + 3.14/2.0, 0.17453292519943295); 
            angle_pub_.publish((float)(angle));
            RoadDetector::visualizeLines();
        }
    double getDist(cv::Vec4i line, double m, double b){ 
        return abs(m*detected_edges_.cols/2 + b - detected_edges_.rows/2)/sqrt(m*m + 1);
    }
    void transformUvToXy(int u, int v, double* x, double* y){
        cv::Vec3d homogeneous_point = cv::Vec3d(u, v, 1.0);
        std::cout << homography_matrix_.type() << std::endl;
        cv::Mat xy_point =  homography_matrix_ * cv::Mat(homogeneous_point);
        double scaling_factor = 1/xy_point.at<double>(2,0);
        cv::Mat homoogeneous_xy = xy_point * scaling_factor;
        *x = homoogeneous_xy.at<double>(1,0);
        *y = -homoogeneous_xy.at<double>(0,0);
    }
    cv::Vec4i generate_lane(double m, double b){
        cv::Vec4i lane; 
        lane[1] = detected_edges_.rows/2; 
        lane[3] = 0;
        lane[0] = std::max(-detected_edges_.cols/2, std::min(detected_edges_.cols, (int)((lane[1] - b)/m)));
        lane[2] = std::max(-detected_edges_.cols/2, std::min(detected_edges_.cols, (int)((lane[3] - b)/m)));
        return lane;
    }
    void visualizeLines(){
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker outer_line_list_;
        visualization_msgs::Marker mid_line_list;
        outer_line_list_.header.frame_id = "/base_link";
        outer_line_list_.header.stamp = ros::Time::now();
        outer_line_list_.ns = "detected_lines";
        outer_line_list_.id = 0;
        outer_line_list_.type = visualization_msgs::Marker::LINE_LIST;
        outer_line_list_.action = visualization_msgs::Marker::ADD;
        outer_line_list_.pose.orientation.w = 1.0;
        outer_line_list_.scale.x = .1;
        
        mid_line_list.header.frame_id = "/base_link";
        mid_line_list.header.stamp = ros::Time::now();
        mid_line_list.ns = "detected_lines";
        mid_line_list.id = 1;
        mid_line_list.type = visualization_msgs::Marker::LINE_LIST;
        mid_line_list.action = visualization_msgs::Marker::ADD;
        mid_line_list.pose.orientation.w = 1.0;
        mid_line_list.scale.x = .1;

        outer_line_list_.color.a = 1.0; // Don't forget to set the alpha!
        outer_line_list_.color.r = 1.0;
        outer_line_list_.color.g = 1.0;
        outer_line_list_.color.b = 1.0;
        
        mid_line_list.color.a = 1.0; // Don't forget to set the alpha!
        mid_line_list.color.r = 0.0;
        mid_line_list.color.g = 1.0;
        mid_line_list.color.b = 0.0;

        // Set marker duration to 1 second
        outer_line_list_.lifetime = ros::Duration(.5);
        mid_line_list.lifetime = ros::Duration(.5);
        double x,y,z;
        z = 0.0;
        RoadDetector::transformUvToXy(left_line_[0], left_line_[1], &x, &y);
        outer_line_list_.points.push_back(RoadDetector::makePoint(x,y,z));
        RoadDetector::transformUvToXy(left_line_[2], left_line_[3], &x, &y);
        outer_line_list_.points.push_back(RoadDetector::makePoint(x,y,z));
        RoadDetector::transformUvToXy(right_line_[0], right_line_[1], &x, &y);
        outer_line_list_.points.push_back(RoadDetector::makePoint(x,y,z));
        RoadDetector::transformUvToXy(right_line_[2], right_line_[3], &x, &y);
        outer_line_list_.points.push_back(RoadDetector::makePoint(x,y,z));
        RoadDetector::transformUvToXy(mid_line_[0], mid_line_[1], &x, &y);
        mid_line_list.points.push_back(RoadDetector::makePoint(x,y,z));
        RoadDetector::transformUvToXy(mid_line_[2], mid_line_[3], &x, &y);
        mid_line_list.points.push_back(RoadDetector::makePoint(x,y,z));
        marker_array.markers.push_back(outer_line_list_);
        marker_array.markers.push_back(mid_line_list);
        marker_pub_.publish(marker_array);
    }
    geometry_msgs::Point makePoint(int x, int y , int z){
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "road_detector");
    ros::NodeHandle nh;
    RoadDetector rd(nh);
    ros::spin();
    return 0;
}
