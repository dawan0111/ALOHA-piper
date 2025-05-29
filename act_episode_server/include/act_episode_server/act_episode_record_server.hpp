#ifndef ACT_EPISODE_RECORD_SERVER_HPP_
#define ACT_EPISODE_RECORD_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_srvs/srv/empty.hpp"

namespace ACT {
class EpisodeRecordServer : public rclcpp::Node {
  using Image = sensor_msgs::msg::CompressedImage;

 public:
  explicit EpisodeRecordServer(const rclcpp::NodeOptions& node_options);

 private:
  void load_parameter();
  void configure_interface();
  void record_start();
  void record_finish();

 private:
  bool is_recoding_{false};

  std::string record_path;
  std::vector<std::string> image_topic_names_;

  std::vector<rclcpp::Subscription<Image>::SharedPtr> image_subs_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr record_service_;
  std::unique_ptr<rosbag2_cpp::Writer> recorder_;
};
}  // namespace ACT

#endif