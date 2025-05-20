#include "rclcpp/rclcpp.hpp"
#include "training_interfaces/msg/person.hpp"

class PersonSubscriber : public rclcpp::Node {
public:
    PersonSubscriber() : Node("person_subscriber") {
        subscription_ = this->create_subscription<training_interfaces::msg::Person>(
            "person_topic", 10,
            [this](const training_interfaces::msg::Person::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received: %s, %d", msg->name.c_str(), msg->age);
            });
    }

private:
    rclcpp::Subscription<training_interfaces::msg::Person>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PersonSubscriber>());
    rclcpp::shutdown();
    return 0;
}
