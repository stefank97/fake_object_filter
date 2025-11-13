#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>


namespace fake_object_filter {

    class FakeObjectFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan> {

        public:
            FakeObjectFilter();
            ~FakeObjectFilter() override = default;
            void setLogger(const rclcpp::Logger &logger) { logger_ = logger; };

            bool configure() override;
            bool update(const sensor_msgs::msg::LaserScan &in, sensor_msgs::msg::LaserScan &out) override;

        private:
            rclcpp::Logger logger_;
            double obj_pos_xA_{0.0};
            double obj_pos_yA_{0.0};
            double obj_pos_xB_{0.0};
            double obj_pos_yB_{0.0};
            double obj_pos_xC_{0.0};
            double obj_pos_yC_{0.0};
            double obj_pos_xD_{0.0};
            double obj_pos_yD_{0.0};
            double obj_pos_z_{0.0};
            double obj_width_ {0.0};
            std::string tf_source_ {"laser"}; 
            std::string tf_target_ {"map"};

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::shared_ptr<rclcpp::Clock> ros_clock_;



    };
};