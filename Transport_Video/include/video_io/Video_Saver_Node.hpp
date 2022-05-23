
#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

class VideoSaverNode : public rclcpp::Node
{
public:
    VideoSaverNode();
    ~VideoSaverNode();

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void quit_node();

    bool first_message;
    bool verbose_logging;
    int fourcc;
    int record_every_nth_frame;
    int skip_counter;
    int quit_after_s_seconds;
    double output_fps;
    bool burn_timestamp;

    float font_scale = 2;
    int thickness = 1;
    int baseline = 0 + thickness;
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];
    cv::Size text_size;

    std::string file_extension;
    std::string image_topic;
    std::string output_filename;
    std::string output_csv_filename;
    std::string codec;
    std::string experiment_folder;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter outputVideo;
    // rclcpp::timer::WallTimer::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::ofstream csv_file;
};
#endif // _VIDEO_PUBLISHER_NODE_H
