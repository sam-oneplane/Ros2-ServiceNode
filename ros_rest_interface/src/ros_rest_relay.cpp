#include <ros_rest_relay.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::vector<std::string> initial_topics = {"topic_1"};
    auto node {std::make_shared<RosRestRelay>(initial_topics)};
    rclcpp::spin(node); // allow the ros node to keep running and check for events on subscribed topics.
    rclcpp::shutdown();
    return 0;
}
