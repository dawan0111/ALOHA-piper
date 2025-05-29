#ifndef ACT_EPISODE_REPLAY_SERVER_HPP_
#define ACT_EPISODE_REPLAY_SERVER_HPP_

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace ACT {
class EpisodeReplayServer : public rclcpp::Node {
  using Image = sensor_msgs::msg::CompressedImage;
  using ImageRaw = sensor_msgs::msg::Image;

 public:
  explicit EpisodeReplayServer(const rclcpp::NodeOptions& node_options);

 private:
  void load_parameter();
  void configure_interface();

 private:
  std::vector<std::string> image_topic_names_;

  std::vector<rclcpp::Subscription<Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Publisher<ImageRaw>::SharedPtr> image_pubs_;
};
}  // namespace ACT

#endif