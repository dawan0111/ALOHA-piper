#include "act_episode_server/act_episode_record_server.hpp"

#include <chrono>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

namespace ACT {

EpisodeRecordServer::EpisodeRecordServer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("episode_record_server", options) {
  load_parameter();
  configure_interface();

  RCLCPP_INFO(this->get_logger(), "EpisodeRecordServer initialized.");
}

void EpisodeRecordServer::load_parameter() {
  std::vector<std::string> topic_names;

  topic_names.push_back("/camera1/image_compressed");
  topic_names.push_back("/camera2/image_compressed");

  this->declare_parameter("image_topic_names", topic_names);
  this->declare_parameter("record_path", std::string("episode_bag"));

  this->get_parameter("image_topic_names", image_topic_names_);
  this->get_parameter("record_path", record_path);

  RCLCPP_INFO(get_logger(), "Record path: %s", record_path.c_str());
}

void EpisodeRecordServer::configure_interface() {
  for (const auto& topic_name : image_topic_names_) {
    auto sub = this->create_subscription<Image>(
        topic_name, rclcpp::SensorDataQoS(),
        [this, topic_name](const Image::SharedPtr msg) {
          if (is_recoding_ && recorder_) {
            recorder_->write(*msg, topic_name, msg->header.stamp);
          }
        });
    image_subs_.push_back(sub);
  }

  record_service_ = this->create_service<std_srvs::srv::Empty>(
      "start_record",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        (void)request;
        (void)response;

        if (is_recoding_) {
          record_finish();
        } else {
          record_start();
        }
      });
}

void EpisodeRecordServer::record_start() {
  std::string time_tag = std::to_string(std::time(nullptr));
  std::string full_path = record_path + "_" + time_tag;

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = full_path;
  storage_options.storage_id = "sqlite3";

  //   rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};

  recorder_ = std::make_unique<rosbag2_cpp::Writer>();
  recorder_->open(storage_options);

  for (const auto& topic_name : image_topic_names_) {
    recorder_->create_topic(
        {topic_name, "sensor_msgs/msg/CompressedImage", "cdr", ""});
  }

  is_recoding_ = true;
  RCLCPP_INFO(this->get_logger(), "Recording started to %s", full_path.c_str());
}
void EpisodeRecordServer::record_finish() {
  is_recoding_ = false;
  recorder_->close();
  recorder_.reset();
}

}  // namespace ACT

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ACT::EpisodeRecordServer)
