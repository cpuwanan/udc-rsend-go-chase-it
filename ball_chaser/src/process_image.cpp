#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include "ball_chaser/DriveToTarget.h"

class Process {
public:
    Process(ros::NodeHandle nh, ros::NodeHandle private_nh) 
        : nh_(nh), private_nh_(private_nh)
    {
        winname_ = ros::this_node::getName();
        
        std::string image_topic;
        private_nh_.param<std::string>("image_topic", image_topic, "/raw_image");
        private_nh_.param<bool>("show_ocv_result", show_ocv_result_, true);
        
        private_nh_.param<std::string>("service_name", service_name_, "/raw_image");
        
        win_width_thresholds_.resize(3);
        speed_thresholds_.resize(4);
        ball_area_threasholds_.resize(2);
        
        this->setXmlRpcData("win_width_thresholds", win_width_thresholds_);
        this->setXmlRpcData("speed_thresholds", speed_thresholds_);
        this->setXmlRpcData("ball_area_threasholds", ball_area_threasholds_);
        
        img_sub_ = nh_.subscribe(image_topic, 1, &Process::processImageCallback, this);
        
        client_ = nh_.serviceClient<ball_chaser::DriveToTarget>(service_name_);
    }
    
    template <class T>
    bool setXmlRpcData(std::string key, std::vector<T>& data) {
        int N = (int)data.size();
        
        XmlRpc::XmlRpcValue read_data;
        if (!private_nh_.getParam(key, read_data)) {
            ROS_ERROR("Failed to set XmlRpcData for '%s'", key.c_str());
        } else {
            if ((int)read_data.size() == N) {
                for (int i=0; i<N; i++) {
                    data[i] = static_cast<T>(read_data[i]);
                }
            } else {
                ROS_ERROR("Invalid data size for setting XmlRpcData '%s'", key.c_str());
            }
        }
    }
    
    void driveRobot(float lin_x, float ang_z) {
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;
        if (client_.call(srv)) {
            ROS_INFO("Requested %.2lf, %.2lf", lin_x, ang_z);
        } else {
            ROS_ERROR("** Client failed to call service '%s'", service_name_.c_str());
        }
        
        ROS_INFO("Speed %.2f, %.2f", lin_x, ang_z);
    }
    
    void processImageCallback(const sensor_msgs::Image img) {
        cv_bridge::CvImagePtr ptr;
        cv::Mat image, segmented_image;
        try {
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            cv::resize(ptr->image, image, cv::Size(), 0.25, 0.25);
            this->detectWhiteColor(image, segmented_image);
            
            cv::Point pt;
            bool found = this->detectContour(image, segmented_image, pt);
            
            if (found) {
                float lin_x, ang_z;
                this->calMotion(lin_x, ang_z, pt, image.cols);
                this->driveRobot(lin_x, ang_z);
            } else {
                this->driveRobot(0, 0);
            }
            
        } catch(cv_bridge::Exception& e) {
            ROS_WARN("cv_bridge exception: %s", e.what());
        }
        
        if (show_ocv_result_) {
            cv::imshow(winname_, image);
            cv::waitKey(5);
        }
    }
    
    void detectWhiteColor(cv::Mat& src, cv::Mat& dst, int threshold = 220) {
        cv::Mat grey(src.size(), CV_8UC1);
        grey.setTo(0);
        for (int j=0; j<src.rows; j++) {
            for (int i=0; i<src.cols; i++) {
                int blue = src.at<cv::Vec3b>(j, i)[0];
                int green = src.at<cv::Vec3b>(j, i)[1];
                int red = src.at<cv::Vec3b>(j, i)[2];

                if (blue > threshold and green > threshold and red > threshold) {
                    grey.at<uchar>(j, i) = 255;
                }
            }
        }
        grey.copyTo(dst);
    }
    
    bool detectContour(cv::Mat& src, cv::Mat& grey, cv::Point& target_pt) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat tmp;
        cv::findContours(grey, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        int selected_contour_index = -1;
        double max_contour_area = 0;
        for (size_t i=0; i<contours.size(); i++) {
            cv::drawContours(src, contours, (int)i, cv::Scalar(0, 255, 0), 1, 8, hierarchy, 0, cv::Point());
            
            double area = cv::contourArea(contours[i]);
            if (area > max_contour_area) {
                max_contour_area = area;
                selected_contour_index = i;
            }
        }
        
        bool found_target = false;
        ball_area_detected_ = 0;
        
        if (selected_contour_index >= 0 and !contours.empty()) {
                   
            ball_area_detected_ = cv::contourArea(contours[selected_contour_index]);
            
            if (ball_area_detected_ > ball_area_threasholds_[Min]) {
                
                ball_area_detected_ = std::min(ball_area_threasholds_[Max], ball_area_detected_); 
                
                cv::Moments mu = cv::moments(contours[selected_contour_index], false);
                cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
                cv::circle(src, mc, 2, cv::Scalar(0, 0, 255), -1, 8, 0);
                target_pt.x = mc.x;
                target_pt.y = mc.y;
                found_target = true;
            } else {
                ball_area_detected_ = 0;
            }
        }
        
        return found_target;
    }
    
    void calMotion(float& lin_x, float& ang_z, cv::Point target_pt, int width) {
        int length = 0;
        bool done = false;
        int win_section = -1;
        for (int i=0; i<(int)win_width_thresholds_.size() and !done; i++) {
            length += (win_width_thresholds_[i] * width);
            
            if (target_pt.x < length) {
                win_section = i;
                done = true;
            }
        }
        
        if (win_section >= 0) {
                        
            // Cal linear speed
            lin_x = speed_thresholds_[LinAbsMin] + (double)(ball_area_threasholds_[Max] - ball_area_detected_) / (double)(ball_area_threasholds_[Max] - ball_area_threasholds_[Min]) * (speed_thresholds_[LinAbsMax] - speed_thresholds_[LinAbsMin]);

            // Cal angular speed
            if (win_section == ForwardWin) {
                ang_z = 0.0;
            } else {
                int dir = (win_section < ForwardWin) ? 1 : -1;
                int d_x = abs(width / 2 - target_pt.x);
                ang_z = speed_thresholds_[AngAbsMin] + (double)(d_x * dir) / (double)(width/2) * (speed_thresholds_[AngAbsMax] - speed_thresholds_[AngAbsMin]);
                
                ROS_INFO("Win section: %d (dir: %d, d_x: %d, ang_z: %.2f)", win_section, dir, d_x, ang_z);
            }
        } else {
            lin_x = 0.0;
            ang_z = 0.0;
        }
    }
    
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber img_sub_;
    ros::ServiceClient client_;
    
    enum {LeftWin, ForwardWin, RightWin};
    std::vector<double> win_width_thresholds_;
    
    enum {LinAbsMin, LinAbsMax, AngAbsMin, AngAbsMax};
    std::vector<double> speed_thresholds_;
    
    std::string service_name_;
    
    enum {Min, Max};
    std::vector<int> ball_area_threasholds_;
    int ball_area_detected_;
    
    bool show_ocv_result_;
    std::string winname_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "process_image");
    ros::NodeHandle nh, private_nh("~");
    
    std::string service_name;
    private_nh.param<std::string>("service_name", service_name, "ball_chaser/command_robot");
    
    Process process(nh, private_nh);
    ros::spin();
    
    return 0;
}
