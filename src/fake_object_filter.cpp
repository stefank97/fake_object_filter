#include "fake_object_filter/fake_object_filter.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/node_options.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuw_geometry/point2d.hpp>
#include <tuw_geometry/line2d.hpp>
#include <tuw_geometry/linesegment2d.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
using namespace std::chrono_literals;


namespace fake_object_filter {

    FakeObjectFilter::FakeObjectFilter() 
    : logger_(rclcpp::get_logger("filter_node_plugin"))
    {};

    bool FakeObjectFilter::configure(){

        bool ok_xA = this->getParam("obj_pos_xA", obj_pos_xA_);
        bool ok_yA = this->getParam("obj_pos_yA", obj_pos_yA_);
        bool ok_xB = this->getParam("obj_pos_xB", obj_pos_xB_);
        bool ok_yB = this->getParam("obj_pos_yB", obj_pos_yB_);
        bool ok_xC = this->getParam("obj_pos_xC", obj_pos_xC_);
        bool ok_yC = this->getParam("obj_pos_yC", obj_pos_yC_);
        bool ok_xD = this->getParam("obj_pos_xD", obj_pos_xD_);
        bool ok_yD = this->getParam("obj_pos_yD", obj_pos_yD_);
        bool ok_z = this->getParam("obj_pos_z", obj_pos_z_);
        bool ok_w = this->getParam("obj_width", obj_width_);
        bool ok_src = this->getParam("tf_source", tf_source_);
        bool ok_tgt = this->getParam("tf_target", tf_target_);

        bool all_ok = ok_xA && ok_yA && ok_xB && ok_yB &&
                            ok_xC && ok_yC && ok_xD && ok_yD &&
                            ok_z  && ok_src && ok_tgt;

        ros_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(ros_clock_);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        

        RCLCPP_INFO_ONCE(
            logger_,
            "Plugin loaded."
        );

        if (!all_ok) {
            RCLCPP_WARN(
                logger_, 
                "Not all parameters set; using defaults where missing."
            );
        }




        return true;
    }

    bool FakeObjectFilter::update(const sensor_msgs::msg::LaserScan &in, sensor_msgs::msg::LaserScan &out){
        //Read parameters again every update circle because of rviz
        this->getParam("obj_pos_xA", obj_pos_xA_);
        this->getParam("obj_pos_yA", obj_pos_yA_);
        this->getParam("obj_pos_xB", obj_pos_xB_);
        this->getParam("obj_pos_yB", obj_pos_yB_);
        this->getParam("obj_pos_xC", obj_pos_xC_);
        this->getParam("obj_pos_yC", obj_pos_yC_);
        this->getParam("obj_pos_xD", obj_pos_xD_);
        this->getParam("obj_pos_yD", obj_pos_yD_);
        this->getParam("obj_pos_z", obj_pos_z_);
        this->getParam("obj_width", obj_width_);

        //lookup map - laser transform
        //work on a copy
        sensor_msgs::msg::LaserScan fscan = in;
        
        geometry_msgs::msg::TransformStamped laser_tf;


        try {
            rclcpp::Time latest_tf(in.header.stamp);
            laser_tf = tf_buffer_->lookupTransform(tf_target_, tf_source_, latest_tf, 0.5s);
        } catch (const std::exception &){
            out = std::move(fscan);
            return true;
        }

        //laser pos in map
        const auto laser_pos_x = laser_tf.transform.translation.x;
        const auto laser_pos_y = laser_tf.transform.translation.y;
        const auto laser_pos_z = laser_tf.transform.translation.z;
        const auto laser_pos_quat = laser_tf.transform.rotation;

        RCLCPP_DEBUG_ONCE(
            logger_,
            "Laser hight is: %f", laser_pos_z
        );

        //ignore objects that are too far above/below the laser (in case of multiple sensors)
        const double obj_height_diff = obj_pos_z_ - laser_pos_z;

        const double max_height_diff = 1;
        if (obj_height_diff > max_height_diff) {
            out = std::move(fscan);
            return true;
        }

        tf2::Quaternion q;
        tf2::fromMsg(laser_pos_quat, q);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        const auto cos_y = std::cos(-yaw);
        const auto sin_y = std::sin(-yaw);

        //translate obj cords into laser frame && rotate object pos into laser frame (inverse rotation to shift world cords - laser cords)
        //A
        const double obj_xA_shifted = obj_pos_xA_ - laser_pos_x;
        const double obj_yA_shifted = obj_pos_yA_ - laser_pos_y;
        const double obj_laser_xA = cos_y * obj_xA_shifted - sin_y * obj_yA_shifted;
        const double obj_laser_yA = sin_y * obj_xA_shifted + cos_y * obj_yA_shifted;
        //B
        const double obj_xB_shifted = obj_pos_xB_ - laser_pos_x;
        const double obj_yB_shifted = obj_pos_yB_ - laser_pos_y;
        const double obj_laser_xB = cos_y * obj_xB_shifted - sin_y * obj_yB_shifted;
        const double obj_laser_yB = sin_y * obj_xB_shifted + cos_y * obj_yB_shifted;
        //C
        const double obj_xC_shifted = obj_pos_xC_ - laser_pos_x;
        const double obj_yC_shifted = obj_pos_yC_ - laser_pos_y;
        const double obj_laser_xC = cos_y * obj_xC_shifted - sin_y * obj_yC_shifted;
        const double obj_laser_yC = sin_y * obj_xC_shifted + cos_y * obj_yC_shifted;
        //D
        const double obj_xD_shifted = obj_pos_xD_ - laser_pos_x;
        const double obj_yD_shifted = obj_pos_yD_ - laser_pos_y;
        const double obj_laser_xD = cos_y * obj_xD_shifted - sin_y * obj_yD_shifted;
        const double obj_laser_yD = sin_y * obj_xD_shifted + cos_y * obj_yD_shifted;


        //Points in laser frame as vector
        tuw::Point2D A(obj_laser_xA, obj_laser_yA);
        tuw::Point2D B(obj_laser_xB, obj_laser_yB);
        tuw::Point2D C(obj_laser_xC, obj_laser_yC);
        tuw::Point2D D(obj_laser_xD, obj_laser_yD);

        //Calculate lines
        tuw::LineSegment2D lineAB(A, B);
        tuw::LineSegment2D lineBC(B, C);
        tuw::LineSegment2D lineCD(C, D);
        tuw::LineSegment2D lineDA(D, A);

        std::array<tuw::LineSegment2D, 4> all_lines = { lineAB, lineBC, lineCD, lineDA };

        //Laser Origin
        tuw::Point2D O(0,0);

        //Find Angle for every beam and check the cross product
        for (size_t i = 0; i < in.ranges.size(); ++i) {
            const double angle = in.angle_min + i * in.angle_increment;
            const double dx = std::cos(angle);
            const double dy = std::sin(angle);
            
            //Set a point in direction of beam & create a infinite line
            tuw::Point2D direction_point(dx, dy);
            tuw::Line2D beam(O, direction_point);

            //for smallest positive value - "First hit of beam" (set to max, so that the new value is deffinitly smaller)
            double current_t = std::numeric_limits<double>::max();
            bool crossing_found = false;

            for (const auto line : all_lines) {
                //Rectangle line as an infinite line
                tuw::Line2D inf_line(line.p0(), line.p1());

                //cross product of the two infinite lines
                tuw::Point2D crossing = beam.intersection(inf_line);

                //Calculate diff between the two points a.k.a. Vector from e.g. A to B
                const double seg_dx = line.p1().x() - line.p0().x();
                const double seg_dy = line.p1().y() - line.p0().y();
                
                //Calculate segment parameter s: is the point on the line or not? ("if" to maybe save myself from division through 0)
                double s;
                if (seg_dx > seg_dy) {
                    s = (crossing.x() - line.p0().x()) / seg_dx;                
                }
                else {
                    s = (crossing.y() - line.p0().y()) / seg_dy; 
                }
        
                //if s is element of [0,1]: Point is on line
                if (s < 0.0 || s > 1.0) {
                    RCLCPP_DEBUG(
                        logger_,
                        "Crossing point is not on relevant line - segment parameter(s) is: %f", s
                    );
                    continue;
                }

                //Distance along the beam from Origin to cross product
                const double t = (crossing.x() * dx + crossing.y() * dy);
                //Take the first "beam hit"
                if (t > 0 && t < current_t) {
                    current_t = t;
                    crossing_found = true;
                }

            }
            //If line got crossed and beam isn't longer than a real sensor distance set
            if (crossing_found && current_t < fscan.ranges[i]) {
                fscan.ranges[i] = current_t;
            }
        }
        fscan.header.stamp = in.header.stamp;
        out = std::move(fscan);
        return true;

    };

}

PLUGINLIB_EXPORT_CLASS(fake_object_filter::FakeObjectFilter,
                       filters::FilterBase<sensor_msgs::msg::LaserScan>)










