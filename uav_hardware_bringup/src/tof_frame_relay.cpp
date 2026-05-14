// tof_frame_relay
//
// The upstream sipeed_tof_ms_a010 driver hardcodes header.frame_id = "tof" for
// every instance. With 5 sensors on the drone, we need each instance's output
// to carry frame_id = "tof_<N>_link" so downstream TF lookups (cloud_merge_node)
// resolve to the correct per-sensor static transform.
//
// One instance of this node per ToF: subscribes to the sipeed node's "depth"
// and "cloud" topics (relative — namespaced by the launch), rewrites the
// frame_id, and republishes on the final /drone/tof_<N>/* topics.

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ToFFrameRelay : public rclcpp::Node {
public:
  ToFFrameRelay() : Node("tof_frame_relay") {
    target_frame_ = declare_parameter("target_frame", std::string("tof_0_link"));
    const std::string depth_in  = declare_parameter("depth_in",  std::string("depth"));
    const std::string cloud_in  = declare_parameter("cloud_in",  std::string("cloud"));
    const std::string depth_out = declare_parameter("depth_out", std::string("/drone/tof_0/depth"));
    const std::string cloud_out = declare_parameter("cloud_out", std::string("/drone/tof_0/points"));

    auto qos = rclcpp::QoS(10).best_effort();
    pub_depth_ = create_publisher<sensor_msgs::msg::Image>(depth_out, qos);
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out, qos);

    sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
        depth_in, qos,
        [this](sensor_msgs::msg::Image::SharedPtr m) {
          m->header.frame_id = target_frame_;
          pub_depth_->publish(*m);
        });

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_in, qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {
          m->header.frame_id = target_frame_;
          pub_cloud_->publish(*m);
        });

    RCLCPP_INFO(get_logger(),
      "tof_frame_relay up: frame='%s'  [%s,%s] -> [%s,%s]",
      target_frame_.c_str(), depth_in.c_str(), cloud_in.c_str(),
      depth_out.c_str(), cloud_out.c_str());
  }

private:
  std::string target_frame_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToFFrameRelay>());
  rclcpp::shutdown();
  return 0;
}
