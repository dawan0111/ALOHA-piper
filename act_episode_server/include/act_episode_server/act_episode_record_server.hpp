#ifndef ACT_EPISODE_RECORD_SERVER_HPP_
#define ACT_EPISODE_RECORD_SERVER_HPP_

#include <mutex>

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

  template <typename MsgT>
  void write_to_bag(const std::string& topic_name, const MsgT& msg,
                    const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (recorder_) {
      recorder_->write(msg, topic_name, stamp);
    }
  }

 private:
  bool is_recoding_{false};

  std::string record_path;
  std::vector<std::string> image_topic_names_;
  std::mutex write_mutex_;

  std::vector<rclcpp::Subscription<Image>::SharedPtr> image_subs_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr record_service_;
  std::unique_ptr<rosbag2_cpp::Writer> recorder_;
};
}  // namespace ACT

#endif