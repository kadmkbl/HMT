
#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

class VideoPublisherNode : public rclcpp::Node
{
public:
    VideoPublisherNode();
    std::string filename;

private:
    void publishImage();
    void convert_frame_to_message(const cv::Mat &frame, sensor_msgs::msg::Image &img_msg);

    bool loop_play;
    bool publish_as_color;
    bool publish_latency;
    bool add_timestamp;
    bool verbose_logging;
    int dt_ms;
    int total_n_frames;
    int width;
    int height;
    int count;
    int start_frame;
    double publish_frequency;
    double downsample_ratio;
    std::string image_topic;
    std::string latency_topic;
    cv::VideoCapture cap;
    cv::Mat frame, gray, resized_frame;

    rclcpp::TimerBase::SharedPtr image_timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    std::shared_ptr<sensor_msgs::msg::Image> img_msg;
};

#endif // _VIDEO_PUBLISHER_NODE_H
