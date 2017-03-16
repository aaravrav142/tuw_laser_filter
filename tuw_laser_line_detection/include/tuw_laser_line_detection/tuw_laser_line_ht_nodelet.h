#ifndef TUW_LASER_LINE_HT_NOTELET_H
#define TUW_LASER_LINE_HT_NOTELET_H

#include <nodelet/nodelet.h>
#include <tuw_laser_line_detection/LaserLineHTConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core/core.hpp>
#include <memory>

#include <ros/ros.h>
namespace tuw_laser_line_detection
{

class LaserLineHTNodelet : public nodelet::Nodelet
{
    static const int FNC_HOUGH_LINES   = 0;
    static const int FNC_HOUGH_LINES_P = 1;
    ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
public:
    virtual void onInit();
    ros::NodeHandle private_nh_;
    ros::Publisher line_pub_;
    cv::Mat img_scan_;
    cv::Mat img_lines_;
    void callbackLaser ( const sensor_msgs::LaserScan::ConstPtr& );   /// callback function to execute on incoming sensor data
      
    tuw_laser_line_detection::LaserLineHTConfig config_;
    dynamic_reconfigure::Server<tuw_laser_line_detection::LaserLineHTConfig> reconfigureServer_; /// parameter server 
    dynamic_reconfigure::Server<tuw_laser_line_detection::LaserLineHTConfig>::CallbackType reconfigureFnc_; /// parameter server stuff general use
    void callbackReconfigure ( tuw_laser_line_detection::LaserLineHTConfig &config, uint32_t level ); /// callback function on incoming parameter changes for general use
};

}


#endif