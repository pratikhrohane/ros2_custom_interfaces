#include "rclcpp/rclcpp.hpp"
#include "training_interfaces/srv/value.hpp"

#include <chrono>
using namespace std::chrono_literals;


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_two_ints_client");
    auto client = node->create_client<training_interfaces::srv::Value>("add_two_ints");

    while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(node->get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<training_interfaces::srv::Value::Request>();
    request->a = 10;
    request->b = 20;

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Sum: %d", result.get()->val);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}
