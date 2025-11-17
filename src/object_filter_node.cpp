#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <filters/filter_chain.hpp>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include "fake_object_filter/fake_object_filter.hpp"


class ObjectFilterNode : public rclcpp::Node {
public:
  ObjectFilterNode()
    : Node(
      "filter_node",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true) ),
    filter_chain_("sensor_msgs::msg::LaserScan")
  {
    auto qos = rclcpp::SensorDataQoS();

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos, std::bind(&ObjectFilterNode::cb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", qos);
    rect_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/fake_object/rectangle", rclcpp::QoS(1), std::bind(&ObjectFilterNode::rectangle_cb, this,std::placeholders::_1));



    //load yaml and initialize filter from parameter-namespace "filters"
    bool ok = filter_chain_.configure("filters",
      this->get_node_logging_interface(),
      this->get_node_parameters_interface());

    RCLCPP_INFO_ONCE(get_logger(), "FilterChain configured = %s", ok ? "true" : "false");

    if (!ok) {
      RCLCPP_WARN_ONCE(get_logger(), "No filters configured under namespace 'filters'.");
    } 
    else {
      RCLCPP_INFO_ONCE(get_logger(), "FilterChain configured.");
    }
  }

  
private:

  void cb(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) {
    sensor_msgs::msg::LaserScan msg_out;

    bool applied = filter_chain_.update(*msg_in, msg_out);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 8000,
      "FilterChain.update() -> %s", applied ? "true" : "false");

    if (applied) {
      pub_->publish(msg_out);
    } else {
      pub_->publish(*msg_in);
    }
  }

  void rectangle_cb(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    latest_rectangle_ = msg;
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_xA", msg->polygon.points[0].x));
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_yA", msg->polygon.points[0].y));
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_xB", msg->polygon.points[1].x));
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_yB", msg->polygon.points[1].y));
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_xC", msg->polygon.points[2].x));
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_yC", msg->polygon.points[2].y));
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_xD", msg->polygon.points[3].x));
    this->set_parameter(rclcpp::Parameter("filters.filter1.params.obj_pos_yD", msg->polygon.points[3].y));

  }

  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr rect_sub_;

  geometry_msgs::msg::PolygonStamped::SharedPtr latest_rectangle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}