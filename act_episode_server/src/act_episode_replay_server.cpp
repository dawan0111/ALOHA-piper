#include "act_episode_server/act_episode_replay_server.hpp"

namespace ACT {

EpisodeReplayServer::EpisodeReplayServer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("episode_replay_server", options) {
  load_parameter();
  configure_interface();
}

void EpisodeReplayServer::load_parameter() {
  this->declare_parameter("image_topic_names", std::vector<std::string>{});
  this->get_parameter("image_topic_names", image_topic_names_);

  RCLCPP_INFO(get_logger(), "Loaded image topics:");
  for (const auto& name : image_topic_names_) {
    RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
  }
}

void EpisodeReplayServer::configure_interface() {
  rclcpp::SensorDataQoS qos;

  for (const auto& topic_name : image_topic_names_) {
    auto cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group;

    std::string pub_topic = topic_name + "/vis";
    auto pub = this->create_publisher<ImageRaw>(pub_topic, qos);
    image_pubs_.push_back(pub);

    auto sub = this->create_subscription<Image>(
        topic_name, qos,
        [this, pub](const Image::SharedPtr msg) {
          try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            auto img_msg = cv_ptr->toImageMsg();

            img_msg->header = msg->header;

            pub->publish(*img_msg);
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to decode compressed image: %s", e.what());
          }
        },
        options);

    image_subs_.push_back(sub);
  }
}

}  // namespace ACT

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ACT::EpisodeReplayServer)