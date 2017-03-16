// this should really be in the implementation (.cpp file)
#include <tuw_laser_line_detection/tuw_laser_line_detection_nodelet.h>
#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wordexp.h>
#include <string>
#include <regex>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS ( tuw_laser_line_detection::LaserLineDetectionNodelet, nodelet::Nodelet )


// Update the input string.
void autoExpandEnvironmentVariables ( std::string & text ) {
    static std::regex env ( "\\$\\{([^}]+)\\}" );
    std::smatch match;
    while ( std::regex_search ( text, match, env ) ) {
        const char * s = getenv ( match[1].str().c_str() );
        const std::string var ( s == NULL ? "" : s );
        text.replace ( match[0].first, match[0].second, var );
    }
}

namespace tuw_laser_line_detection {
void LaserLineDetectionNodelet::onInit() {
    NODELET_DEBUG ( "Initializing nodelet..." );
    private_nh_ = getNodeHandle();
    /// subscribes to laser
    sub_laser_ = private_nh_.subscribe ( "scan", 1, &LaserLineDetectionNodelet::callbackLaser, this );

    reconfigureFnc_ = boost::bind ( &LaserLineDetectionNodelet::callbackReconfigure, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
    line_pub_ = private_nh_.advertise<tuw_geometry_msgs::LineSegments> ( "line_segments", 1000 );
}


void LaserLineDetectionNodelet::callbackReconfigure ( tuw_laser_line_detection::LaserLineDetectionConfig &config, uint32_t level ) {
    NODELET_DEBUG ( "callbackReconfigure LaserLineDetectionConfig!" );
    config_ = config;
}

void LaserLineDetectionNodelet::callbackLaser ( const sensor_msgs::LaserScan& input_scan){
    using namespace cv;
    size_t nr =  input_scan.ranges.size();
    double scale = config_.scan_pixel_resolution;
    double width = scale * ( input_scan.range_max * 2.1 );
    double center = width/2;
    img_scan_.create ( width, width, CV_8U );
    img_scan_.setTo ( 0 );
    cv::Point p;
    for ( size_t i = 0; i < nr; i++ ) {
        double length = input_scan.ranges[i];

        if ( ( length < input_scan.range_max ) && std::isfinite ( length ) ) {
            double angle  = input_scan.angle_min + ( input_scan.angle_increment * i );
            p.x = round ( -cos ( angle ) * length * scale + center );
            p.y = round ( -sin ( angle ) * length * scale + center );
            img_scan_.at<uchar> ( p.x, p.y ) = 255;
        }

    }
    std::vector<Vec4i> lines;
    switch ( config_.line_detection_function ) {
    case FNC_HOUGH_LINES: {
        std::vector<Vec2f> l;
        Vec4i v;
        cv::HoughLines ( img_scan_, l, config_.rho, config_.theta, config_.threshold, 0, 0 );

        for ( size_t i = 0; i < l.size(); i++ ) {
            float rho = l[i][0], theta = l[i][1];
            Point pt1, pt2;
            double a = cos ( theta ), b = sin ( theta );
            double x0 = a*rho, y0 = b*rho;
            v[0] = cvRound ( x0 + 1000* ( -b ) );
            v[1] = cvRound ( y0 + 1000* ( a ) );
            v[2] = cvRound ( x0 - 1000* ( -b ) );
            v[3] = cvRound ( y0 - 1000* ( a ) );
            lines.push_back ( v );
        }
    }
    break;
    case FNC_HOUGH_LINES_P:
    default:
        HoughLinesP ( img_scan_, lines, config_.rho, config_.theta, config_.threshold, config_.min_line_length*scale, config_.max_line_gap*scale );
        break;
    }
    if ( config_.plot_detected_lines ) {
        cvtColor ( img_scan_, img_lines_, CV_GRAY2BGR );
        for ( size_t i = 0; i < lines.size(); i++ ) {
            Vec4i l = lines[i];
            line ( img_lines_, Point ( l[0], l[1] ), Point ( l[2], l[3] ), Scalar ( 0,0,255 ), 3, CV_AA );
        }
        imshow ( "detected lines", img_lines_ );
        waitKey ( 10 );
    }


    tuw_geometry_msgs::LineSegment line_segment_msg;
    tuw_geometry_msgs::LineSegments line_segments_msg;
    for ( int i = 0; i < lines.size(); i++ ) {
        Vec4i l = lines[i];
        line_segment_msg.p0.x = - ( l[1] - center ) / scale;
        line_segment_msg.p0.y = - ( l[0] - center ) / scale;
        line_segment_msg.p0.z = 0;
        line_segment_msg.p1.x = - ( l[3] - center ) / scale;
        line_segment_msg.p1.y = - ( l[2] - center ) / scale;
        line_segment_msg.p1.z = 0;
        line_segments_msg.segments.push_back ( line_segment_msg );
    }
    // set header information
    line_segments_msg.header = input_scan.header;

    line_pub_.publish ( line_segments_msg );
}

}

