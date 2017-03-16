#ifndef TUW_LASER_LINE_SM_NOTELET_H
#define TUW_LASER_LINE_SM_NOTELET_H

#include <nodelet/nodelet.h>
#include <tuw_laser_line_detection/LaserLineSMConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_geometry/measurement_laser.h>
#include <tuw_geometry/linesegment2d_detector.h>
#include <memory>

#include <ros/ros.h>
namespace tuw_laser_line_detection
{

class LaserLineSMNodelet : public nodelet::Nodelet
{
    ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
public:
    virtual void onInit();
    ros::NodeHandle private_nh_;
    ros::Publisher line_pub_;
    tuw::LineSegment2DDetector line_detector_;
    tuw::MeasurementLaserPtr measurement_laser_;  /// laser measurements
    std::vector<tuw::Point2D> measurement_local_scanpoints_; /// laser beam endpoints for line detection
    std::vector<tuw::LineSegment2D> measurement_linesegments_;    /// detected line segments in sensor coordinates
    void callbackLaser ( const sensor_msgs::LaserScan& );   /// callback function to execute on incoming sensor data
      
    dynamic_reconfigure::Server<tuw_laser_line_detection::LaserLineSMConfig> reconfigureServer_; /// parameter server 
    dynamic_reconfigure::Server<tuw_laser_line_detection::LaserLineSMConfig>::CallbackType reconfigureFnc_; /// parameter server stuff general use
    void callbackReconfigure ( tuw_laser_line_detection::LaserLineSMConfig &config, uint32_t level ); /// callback function on incoming parameter changes for general use
};

}


#endif