// ================== UPDATED seecam.cpp ==================
#include "seecam.hpp"
#include <algorithm>  // for std::max / std::min

double SeeCam::get_timestamp() {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string SeeCam::get_local_ip() {
    int sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock == -1) return "Unknown";
    sockaddr_in loopback;
    std::memset(&loopback, 0, sizeof(loopback));
    loopback.sin_family = AF_INET;
    loopback.sin_addr.s_addr = inet_addr("8.8.8.8");
    loopback.sin_port = htons(80);
    if (connect(sock, (sockaddr*)&loopback, sizeof(loopback)) == -1) {
        close(sock);
        return "Unknown";
    }
    socklen_t len = sizeof(loopback);
    getsockname(sock, (sockaddr*)&loopback, &len);
    std::string ip = inet_ntoa(loopback.sin_addr);
    close(sock);
    return ip;
}

void SeeCam::set_jetson_performance() {
    system("sudo nvpmodel -m 0");
    system("sudo jetson_clocks");
    std::cout << "Jetson performance mode enabled." << std::endl;
}

bool SeeCam::load_config() {
    std::ifstream cfg("config.cfg");
    if (!cfg) {
        std::cout << "[ERROR] Failed to load config.cfg, using defaults" << std::endl;
        original_width = 1920; original_height = 1200;
        left_crop = 164; right_crop = 74; top_crop = 0; bottom_crop = 0;
        width = 640; height = 480;
        quality = 80;
        show_frame = false;
        zenoh_key = "seecam/image";
        zenoh_port = 4556;
        return false;
    }

    std::map<std::string, std::string> conf;
    std::string line;
    while (std::getline(cfg, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string key = line.substr(0, pos);
        std::string val = line.substr(pos + 1);
        conf[key] = val;
    }

    try { original_width = std::stoi(conf["original_width"]); } catch (...) { original_width = 1920; }
    try { original_height = std::stoi(conf["original_height"]); } catch (...) { original_height = 1200; }
    try { left_crop = std::stoi(conf["left_crop"]); } catch (...) { left_crop = 164; }
    try { right_crop = std::stoi(conf["right_crop"]); } catch (...) { right_crop = 74; }
    try { top_crop = std::stoi(conf["top_crop"]); } catch (...) { top_crop = 0; }
    try { bottom_crop = std::stoi(conf["bottom_crop"]); } catch (...) { bottom_crop = 0; }
    try { width = std::stoi(conf["width"]); } catch (...) { width = 640; }
    try { height = std::stoi(conf["height"]); } catch (...) { height = 480; }
    try { quality = std::stoi(conf["compression_quality"]); } catch (...) { quality = 80; }

    show_frame = (conf.count("SHOW_FRAME") && conf["SHOW_FRAME"] == "true");
    zenoh_key = conf.count("zenoh_key") ? conf["zenoh_key"] : "seecam/image";
    try { zenoh_port = std::stoi(conf["zenoh_port"]); } catch (...) { zenoh_port = 4556; }

    std::cout << "[INFO] Loaded config: Capture " << original_width << "x" << original_height
              << " | Crop L:" << left_crop << " R:" << right_crop
              << " T:" << top_crop << " B:" << bottom_crop
              << " → Output " << width << "x" << height << std::endl;
    return true;
}

cv::VideoCapture SeeCam::init_camera() {
    // Exact GStreamer pipeline from Python viewer (high-res MJPEG)
    std::string gst_pipeline = "v4l2src device=/dev/video0 ! image/jpeg,width=" +
                               std::to_string(original_width) + ",height=" +
                               std::to_string(original_height) +
                               ",framerate=60/1 ! jpegdec ! videoconvert ! appsink";

    cv::VideoCapture c(gst_pipeline, cv::CAP_GSTREAMER);
    if (c.isOpened()) {
        std::cout << "[INFO] Camera opened with GStreamer at " << original_width << "x" << original_height << std::endl;
        return c;
    }

    std::cout << "[WARN] GStreamer failed -> trying normal OpenCV V4L2..." << std::endl;
    c.open(0, cv::CAP_V4L2);
    if (c.isOpened()) {
        c.set(cv::CAP_PROP_FRAME_WIDTH, original_width);
        c.set(cv::CAP_PROP_FRAME_HEIGHT, original_height);
        c.set(cv::CAP_PROP_FPS, 60);
        std::cout << "[INFO] Camera opened with V4L2 backend." << std::endl;
    } else {
        std::cout << "[ERROR] Could not open camera." << std::endl;
    }
    return c;
}

std::vector<uint8_t> SeeCam::encode_image(const cv::Mat& image) {
    std::vector<uint8_t> buffer;
    std::vector<int> encode_param = {cv::IMWRITE_JPEG_QUALITY, quality};
    bool success = cv::imencode(".jpg", image, buffer, encode_param);
    if (!success) {
        std::cout << "[ERROR] Failed to encode image to JPEG" << std::endl;
        return {};
    }
    return buffer;
}

double SeeCam::get_memory_mb() {
    std::ifstream file("/proc/self/status");
    if (!file) return 0.0;
    std::string line;
    while (std::getline(file, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            std::istringstream iss(line.substr(6));
            double mem_kb;
            iss >> mem_kb;
            return mem_kb / 1024.0;
        }
    }
    return 0.0;
}

SeeCam::SeeCam() : show_frame(false), width(640), height(480), quality(80),
                   zenoh_port(4556), zenoh_key("seecam/image"), sequence(0),
                   last_publish_time(0.0),
                   original_width(1920), original_height(1200),
                   left_crop(164), right_crop(74), top_crop(0), bottom_crop(0) {
    load_config();
    set_jetson_performance();
    cap = init_camera();
    if (!cap.isOpened()) {
        std::cout << "[ERROR] Camera not opened." << std::endl;
        return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    zenoh::Config zconfig = zenoh::Config::create_default();
    std::string endpoint = "udp/0.0.0.0:" + std::to_string(zenoh_port);
    zconfig.insert_json5("listen/endpoints", "[\"" + endpoint + "\"]");
    zconfig.insert_json5("scouting/multicast/enabled", "true");
    try {
        session.emplace(std::move(zconfig));
        std::cout << "[INFO] Zenoh session established" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Failed to establish Zenoh session: " << e.what() << std::endl;
        return;
    }
    try {
        pub.emplace(session->declare_publisher(zenoh_key));
        std::cout << "[INFO] Publisher declared on key: " << zenoh_key << std::endl;
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Failed to declare publisher: " << e.what() << std::endl;
        return;
    }
    std::string publisher_ip = get_local_ip();
    std::cout << "[INFO] Publisher running on IP: " << publisher_ip << ":" << zenoh_port << std::endl;
}

SeeCam::~SeeCam() {
    cap.release();
    cv::destroyAllWindows();
    if (session.has_value()) {
        session.reset();
        std::cout << "[INFO] Zenoh session closed" << std::endl;
    }
}

void SeeCam::run() {
    if (!pub.has_value() || !session.has_value()) {
        std::cout << "[ERROR] Publisher or session not initialized." << std::endl;
        return;
    }
    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cout << "[ERROR] Failed to capture frame." << std::endl;
            continue;
        }

        // ====================== CROP (exact match to Python viewer) ======================
        int crop_w = original_width - left_crop - right_crop;
        int crop_h = original_height - top_crop - bottom_crop;
        crop_w = std::max(100, crop_w);
        crop_h = std::max(100, crop_h);

        // Safety clamp (prevents out-of-bounds like Python's clamp logic)
        int x1 = std::max(0, std::min(left_crop, frame.cols - crop_w));
        int y1 = std::max(0, std::min(top_crop, frame.rows - crop_h));

        cv::Mat cropped = frame(cv::Rect(x1, y1, crop_w, crop_h));

        // ====================== RESIZE TO 640x480 (exact match to Python) ======================
        cv::Mat final_frame;
        cv::resize(cropped, final_frame, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);

        // Match original C++ publisher RGB conversion
        cv::cvtColor(final_frame, final_frame, cv::COLOR_BGR2RGB);

        if (show_frame) {
            cv::imshow("SeeCam Publisher - " + std::to_string(width) + "x" + std::to_string(height), final_frame);
            if (cv::waitKey(1) == 'q') {
                std::cout << "Pressed 'q'. Exiting..." << std::endl;
                break;
            }
        }

        double current_time = get_timestamp();
        if (current_time - last_publish_time < publish_interval) {
            continue;
        }
        last_publish_time = current_time;

        auto encoded_image = encode_image(final_frame);
        if (encoded_image.empty()) continue;

        double timestamp = current_time;
        uint32_t image_len = static_cast<uint32_t>(encoded_image.size());

        std::vector<uint8_t> message(sizeof(uint32_t) + sizeof(double) + sizeof(uint32_t) + image_len);
        uint8_t* ptr = message.data();

        uint32_t seq_be = htobe32(sequence);
        memcpy(ptr, &seq_be, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        uint64_t ts_bits = *reinterpret_cast<const uint64_t*>(&timestamp);
        uint64_t ts_be = htobe64(ts_bits);
        memcpy(ptr, &ts_be, sizeof(uint64_t));
        ptr += sizeof(uint64_t);

        uint32_t len_be = htobe32(image_len);
        memcpy(ptr, &len_be, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        memcpy(ptr, encoded_image.data(), image_len);

        pub->put(zenoh::Bytes(std::move(message)));

        std::cout << std::fixed << std::setprecision(6)
                  << "[INFO] Published image #" << sequence << " at " << timestamp
                  << " (" << width << "x" << height << "), Memory: " << get_memory_mb() << " MB" << std::endl;
        sequence++;
    }
}
