#include "rclcpp/rclcpp.hpp"
#include "training_interfaces/srv/value.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServer : public rclcpp::Node {
public:
    AddTwoIntsServer() : Node("add_two_ints_server") {
        server_ = this->create_service<training_interfaces::srv::Value>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::handle_add, this, _1, _2));
    }

private:
    void handle_add(const std::shared_ptr<training_interfaces::srv::Value::Request> request,
                    std::shared_ptr<training_interfaces::srv::Value::Response> response) {
        response->val = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Received a: %d, b: %d -> val: %d", request->a, request->b, response->val);
    }

    rclcpp::Service<training_interfaces::srv::Value>::SharedPtr server_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddTwoIntsServer>());
    rclcpp::shutdown();
    return 0;
}
