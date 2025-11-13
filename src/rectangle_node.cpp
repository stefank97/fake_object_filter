#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <cmath>

class RectangleNode : public rclcpp::Node{
//TODO: Evtl. Funktion aus Punktberechnung machen.
public:
    RectangleNode()
    : Node(
        "rectangle_node"
        //width, height, yaw_
    )
    {
        this->declare_parameter("width", 1.0);
        this->declare_parameter("height", 1.0);
        this->declare_parameter("yaw", 0.0);

        width_ = this->get_parameter("width").as_double();
        height_ = this->get_parameter("height").as_double();
        yaw_ = this->get_parameter("yaw").as_double();


        sub_ = create_subscription<geometry_msgs::msg::PointStamped>("clicked_point", 10, std::bind(&RectangleNode::cb, this, std::placeholders::_1));

        pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/fake_object/rectangle", 10);
    }
private:
    void cb(const geometry_msgs::msg::PointStamped::SharedPtr msg){
        //get a central point where the rectangle is later all around it
        const double central_x = msg->point.x;
        const double central_y = msg->point.y;
        const double half_width = width_ / 2;
        const double half_height = height_ / 2;

        const double cos = std::cos(yaw_);
        const double sin = std::sin(yaw_);

        geometry_msgs::msg::PolygonStamped polygon;
        geometry_msgs::msg::Point32 p;

        polygon.header.frame_id = msg->header.frame_id;
        polygon.header.stamp = msg->header.stamp;
        
        //Delete old points (Just to be sure)
        polygon.polygon.points.clear();

        //Rotation-matrix + central point bc its not in the origin
        //Point A (+half_width,+half_height)
        p.x = central_x + cos * half_width - sin * half_height;
        p.y = central_y + sin * half_width + cos * half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);


        //point B (-half_width,+half_height)
        p.x = central_x + cos * -half_width - sin * half_height;
        p.y = central_y + sin * -half_width + cos * half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);


        //Point C (-half_width,-half_height)
        p.x = central_x + cos * -half_width - sin * -half_height;
        p.y = central_y + sin * -half_width + cos * -half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);


        //Point D (+half_width,-half_height)
        p.x = central_x + cos * half_width - sin * -half_height;
        p.y = central_y + sin * half_width + cos * -half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);

        RCLCPP_INFO(
            get_logger(),
            "Set Rectangle to Point (%f,%f)", 
            central_x, central_y
        );
        RCLCPP_INFO(
            get_logger(),
            "Rectangle points: A=(%f,%f), B=(%f,%f), C=(%f,%f), D=(%f,%f)", 
            polygon.polygon.points[0].x, polygon.polygon.points[0].y,
            polygon.polygon.points[1].x, polygon.polygon.points[1].y,
            polygon.polygon.points[2].x, polygon.polygon.points[2].y,
            polygon.polygon.points[3].x, polygon.polygon.points[3].y
        );

        pub_->publish(polygon);
        

    }
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;

    double width_{1.0};
    double height_{1.0};
    double yaw_{0.0};
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectangleNode>());
  rclcpp::shutdown();
  return 0;
}
