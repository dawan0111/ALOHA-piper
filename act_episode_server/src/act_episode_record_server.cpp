#include "act_episode_server/act_episode_record_server.hpp"

#include <chrono>
#include <ctime>
#include <string>
#include <unordered_map>

#include "rcpputils/asserts.hpp"

namespace ACT {

EpisodeRecordServer::EpisodeRecordServer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("episode_record_server", options) {
  load_parameter();
  configure_interface();
  RCLCPP_INFO(get_logger(), "EpisodeRecordServer initialized.");
}

void EpisodeRecordServer::load_parameter() {
  this->declare_parameter("topic_names", std::vector<std::string>{});
  this->declare_parameter("record_path", std::string("episode_bag"));

  this->get_parameter("topic_names", topic_names_);
  this->get_parameter("record_path", record_path);

  RCLCPP_INFO(get_logger(), "Record path: %s", record_path.c_str());
  RCLCPP_INFO(get_logger(), "Recording topics:");
  for (const auto& topic : topic_names_) {
    RCLCPP_INFO(get_logger(), "  - %s", topic.c_str());
  }
}

void EpisodeRecordServer::configure_interface() {
  auto topic_type_map = this->get_topic_names_and_types();
  rclcpp::SensorDataQoS qos;

  for (const auto& topic_name : topic_names_) {
    std::string type_name;

    auto it = topic_type_map.find(topic_name);
    if (it != topic_type_map.end() && !it->second.empty()) {
      type_name = it->second[0];
    } else {
      RCLCPP_ERROR(this->get_logger(), "Cannot determine type for topic: %s",
                   topic_name.c_str());
      continue;
    }

    auto generic_sub = this->create_generic_subscription(
        topic_name, type_name, qos,
        [this, topic_name,
         type_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          if (is_recoding_ && recorder_) {
            std::lock_guard<std::mutex> lock(write_mutex_);
            recorder_->write(msg, topic_name, type_name, this->now());
          }
        });

    generic_subs_.push_back(generic_sub);

    RCLCPP_INFO(this->get_logger(), "Subscribed: %s [%s]", topic_name.c_str(),
                type_name.c_str());
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
  std::string full_path = record_path + "/episode_" + time_tag;

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = full_path;
  storage_options.storage_id = "sqlite3";

  recorder_ = std::make_unique<rosbag2_cpp::Writer>();
  recorder_->open(storage_options);

  // 토픽 타입 다시 조회
  auto topic_type_map = this->get_topic_names_and_types();

  for (const auto& topic_name : topic_names_) {
    auto it = topic_type_map.find(topic_name);
    if (it == topic_type_map.end() || it->second.empty()) {
      RCLCPP_WARN(this->get_logger(), "Skipping unknown topic: %s",
                  topic_name.c_str());
      continue;
    }

    recorder_->create_topic({
        topic_name,
        it->second[0],  // 타입
        rmw_get_serialization_format(),
        ""  // default QoS profile
    });
  }

  is_recoding_ = true;
  RCLCPP_INFO(this->get_logger(), "Recording started to %s", full_path.c_str());
}

void EpisodeRecordServer::record_finish() {
  is_recoding_ = false;
  recorder_->close();
  recorder_.reset();
  RCLCPP_INFO(this->get_logger(), "Recording finished.");
}

}  // namespace ACT

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ACT::EpisodeRecordServer)
