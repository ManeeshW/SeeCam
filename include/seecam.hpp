// ================== UPDATED seecam.hpp ==================
#ifndef SEECAM_HPP
#define SEECAM_HPP
#include <opencv2/opencv.hpp>
#include <zenoh.hxx>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <endian.h>
#include <sstream>
#include <iomanip>
#include <optional>

class SeeCam {
private:
    cv::VideoCapture cap;
    std::optional<zenoh::Session> session;
    std::optional<zenoh::Publisher> pub;
    bool show_frame;
    int width, height, quality, zenoh_port;
    int original_width, original_height;
    int left_crop, right_crop, top_crop, bottom_crop;
    std::string zenoh_key;
    uint32_t sequence;
    double last_publish_time;
    const double publish_interval = 0.1;

    double get_timestamp();
    std::string get_local_ip();
    void set_jetson_performance();
    bool load_config();
    cv::VideoCapture init_camera();
    std::vector<uint8_t> encode_image(const cv::Mat& frame);
    double get_memory_mb();

public:
    SeeCam();
    ~SeeCam();
    void run();
};
#endif // SEECAM_HPP
