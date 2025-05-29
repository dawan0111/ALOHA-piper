#ifndef ACT_EPISODE_RECORD_SERVER_HPP_
#define ACT_EPISODE_RECORD_SERVER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "std_srvs/srv/empty.hpp"

namespace ACT {
class EpisodeRecordServer : public rclcpp::Node {
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
  std::vector<std::string> topic_names_;

  std::mutex write_mutex_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subs_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr record_service_;
  std::unique_ptr<rosbag2_cpp::Writer> recorder_;
};
}  // namespace ACT

#endif