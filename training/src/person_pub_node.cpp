#include "rclcpp/rclcpp.hpp"
#include "training_interfaces/msg/person.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PersonPublisher : public rclcpp::Node {
public:
    PersonPublisher() : Node("person_publisher") {
        publisher_ = this->create_publisher<training_interfaces::msg::Person>("person_topic", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&PersonPublisher::publish_msg, this));
    }

private:
    void publish_msg() {
        auto msg = training_interfaces::msg::Person();
        msg.name = "Allien";
        msg.age = 5;
        RCLCPP_INFO(this->get_logger(), "Publishing: %s, %d", msg.name.c_str(), msg.age);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<training_interfaces::msg::Person>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PersonPublisher>());
    rclcpp::shutdown();
    return 0;
}
