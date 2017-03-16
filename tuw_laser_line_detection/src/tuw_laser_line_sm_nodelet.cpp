// this should really be in the implementation (.cpp file)
#include <tuw_laser_line_detection/tuw_laser_line_sm_nodelet.h>
#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <wordexp.h>
#include <string>
#include <regex>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS ( tuw_laser_line_detection::LaserLineSMNodelet, nodelet::Nodelet )


namespace tuw_laser_line_detection {
void LaserLineSMNodelet::onInit() {
    NODELET_INFO ( "Initializing nodelet LaserLineSMNodelet..." );
    private_nh_ = getNodeHandle();
    measurement_laser_ = std::make_shared<tuw::MeasurementLaser>();

    /// subscribes to laser
    sub_laser_ = private_nh_.subscribe ( "scan", 1, &LaserLineSMNodelet::callbackLaser, this );

    reconfigureFnc_ = boost::bind ( &LaserLineSMNodelet::callbackReconfigure, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
    line_pub_ = private_nh_.advertise<tuw_geometry_msgs::LineSegments> ( "line_segments", 1000 );
}


void LaserLineSMNodelet::callbackReconfigure ( tuw_laser_line_detection::LaserLineSMConfig &config, uint32_t level ) {
    NODELET_INFO ( "callbackReconfigure LaserLineSMConfig!" );
  line_detector_.config_.threshold_split_neighbor = config.line_dection_split_neighbor;
  line_detector_.config_.threshold_split = config.line_dection_split_threshold;
  line_detector_.config_.min_length = config.line_dection_min_length;
  line_detector_.config_.min_points_per_line = config.line_dection_min_points_per_line;
  line_detector_.config_.min_points_per_unit = config.line_dection_min_points_per_unit;
}

void LaserLineSMNodelet::callbackLaser ( const sensor_msgs::LaserScan::ConstPtr& input_scan ) {
    NODELET_INFO ( "callbackLaser LaserLineSMNodelet!" );
    int nr = ( input_scan->angle_max - input_scan->angle_min ) / input_scan->angle_increment;
    measurement_laser_->range_max() = input_scan->range_max;
    measurement_laser_->range_min() = input_scan->range_min;
    measurement_laser_->resize ( nr );
    measurement_laser_->stamp() = input_scan->header.stamp.toBoost();
    for ( int i = 0; i < nr; i++ ) {
        tuw::MeasurementLaser::Beam& beam = measurement_laser_->operator[] ( i );
        beam.length = input_scan->ranges[i];
        beam.angle = input_scan->angle_min + ( input_scan->angle_increment * i );
        beam.end_point.x() = cos ( beam.angle ) * beam.length;
        beam.end_point.y() = sin ( beam.angle ) * beam.length;
    }

    measurement_local_scanpoints_.resize ( measurement_laser_->size() );
    for ( size_t i = 0; i < measurement_laser_->size(); i++ ) {
        measurement_local_scanpoints_[i] = measurement_laser_->operator[] ( i ).end_point;
    }
    measurement_linesegments_.clear();
    line_detector_.start ( measurement_local_scanpoints_, measurement_linesegments_ );

    // publish found line segments
    tuw_geometry_msgs::LineSegment line_segment_msg;
    tuw_geometry_msgs::LineSegments line_segments_msg;
    for ( int i = 0; i < measurement_linesegments_.size(); i++ ) {
        line_segment_msg.p0.x = measurement_linesegments_[i].p0().x();
        line_segment_msg.p0.y = measurement_linesegments_[i].p0().y();
        line_segment_msg.p0.z = 0;
        line_segment_msg.p1.x = measurement_linesegments_[i].p1().x();
        line_segment_msg.p1.y = measurement_linesegments_[i].p1().y();
        line_segment_msg.p1.z = 0;
        line_segments_msg.segments.push_back ( line_segment_msg );
    }
    // set header information
    line_segments_msg.header = input_scan->header;

    line_pub_.publish ( line_segments_msg );
}

}

