#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "video_io/msg/burst_record_command.hpp"

class BurstCommandPublisherNode : public rclcpp::Node
{
public:
    BurstCommandPublisherNode()
        : Node("burst_command_publisher")
    {
        std::string burst_record_command_topic = this->declare_parameter<std::string>("burst_record_command_topic", "video_player/burst_record_commands");
        record_duration = this->declare_parameter<double>("record_duration_s", 1.0);

        publisher_ = this->create_publisher<video_io::msg::BurstRecordCommand>(burst_record_command_topic, 10);

        double burst_interval_s = this->declare_parameter<double>("burst_interval_s", 10.0);
        int burst_interval_ms = (int)(burst_interval_s * 1000);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(burst_interval_ms), std::bind(&BurstCommandPublisherNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto burst_record_msg = video_io::msg::BurstRecordCommand();
        burst_record_msg.header.stamp = this->get_clock()->now();
        burst_record_msg.record_duration_s = record_duration;
        publisher_->publish(burst_record_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<video_io::msg::BurstRecordCommand>::SharedPtr publisher_;
    double record_duration;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BurstCommandPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
