#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <linux/hidraw.h>
#include <dirent.h>
#include <cstring>

#define BUFFER_LENGTH 65
#define CAMERA_CONTROL_24CUG 0xA8
#define SET_STREAM_MODE_24CUG 0x1C
#define SET_EXPOSURE_24CUG 0x12
#define MODE_MASTER 0x00
#define MODE_TRIGGER 0x01
#define SET_SUCCESS 0x01
#define SET_FAIL 0x00
#define OS_CODE 0x70
#define LINUX_OS 0x01
#define ECON_VENDOR_ID 0x2560
#define EXPOSURE_MIN 50
#define EXPOSURE_MAX 100000000
#define TIMEOUT_MS 2000

static int findHidDevice() {
    DIR* dir = opendir("/dev");
    if (!dir) return -1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "hidraw", 6) != 0) continue;

        std::string path = std::string("/dev/") + entry->d_name;
        int fd = open(path.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;

        struct hidraw_devinfo info;
        if (ioctl(fd, HIDIOCGRAWINFO, &info) >= 0 && info.vendor == ECON_VENDOR_ID) {
            closedir(dir);
            return fd;
        }
        close(fd);
    }
    closedir(dir);
    return -1;
}

static bool sendOSCode(int hid_fd) {
    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = OS_CODE;    // Report Number for OS identification
    out_buf[2] = LINUX_OS;   // Linux OS identifier

    if (write(hid_fd, out_buf, BUFFER_LENGTH) < 0) {
        return false;
    }

    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(hid_fd, &rfds);
    tv.tv_sec = TIMEOUT_MS / 1000;
    tv.tv_usec = (TIMEOUT_MS % 1000) * 1000;

    if (select(hid_fd + 1, &rfds, NULL, NULL, &tv) <= 0) return false;
    if (read(hid_fd, in_buf, BUFFER_LENGTH) < 0) return false;

    return (in_buf[0] == OS_CODE &&
            in_buf[1] == LINUX_OS &&
            in_buf[2] == SET_SUCCESS);
}

static bool setExposure(int hid_fd, unsigned int exposure_us) {
    if (exposure_us < EXPOSURE_MIN || exposure_us > EXPOSURE_MAX) return false;

    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = CAMERA_CONTROL_24CUG;
    out_buf[2] = SET_EXPOSURE_24CUG;
    out_buf[3] = (exposure_us >> 24) & 0xFF;
    out_buf[4] = (exposure_us >> 16) & 0xFF;
    out_buf[5] = (exposure_us >> 8) & 0xFF;
    out_buf[6] = exposure_us & 0xFF;

    if (write(hid_fd, out_buf, BUFFER_LENGTH) < 0) return false;

    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(hid_fd, &rfds);
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    if (select(hid_fd + 1, &rfds, NULL, NULL, &tv) <= 0) return false;
    if (read(hid_fd, in_buf, BUFFER_LENGTH) < 0) return false;

    return (in_buf[0] == CAMERA_CONTROL_24CUG &&
            in_buf[1] == SET_EXPOSURE_24CUG &&
            in_buf[6] == SET_SUCCESS);
}

static bool setTriggerMode(int hid_fd, bool trigger_mode, bool auto_lock) {
    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = CAMERA_CONTROL_24CUG;
    out_buf[2] = SET_STREAM_MODE_24CUG;
    out_buf[3] = trigger_mode ? MODE_TRIGGER : MODE_MASTER;
    out_buf[4] = auto_lock ? 1 : 0;

    if (write(hid_fd, out_buf, BUFFER_LENGTH) < 0) return false;

    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(hid_fd, &rfds);
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    if (select(hid_fd + 1, &rfds, NULL, NULL, &tv) <= 0) return false;
    if (read(hid_fd, in_buf, BUFFER_LENGTH) < 0) return false;

    return (in_buf[0] == CAMERA_CONTROL_24CUG &&
            in_buf[1] == SET_STREAM_MODE_24CUG &&
            in_buf[6] == SET_SUCCESS);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trigger_cam");
    ros::NodeHandle nh("~");

    std::string device, topic, frame_id, format;
    int width, height, jpeg_quality, exposure_us;
    double fps;
    bool trigger_mode, auto_lock, publish_raw, publish_compressed;

    nh.param("device", device, std::string("/dev/video0"));
    nh.param("topic", topic, std::string("image_raw"));
    nh.param("frame_id", frame_id, std::string("camera"));
    nh.param("width", width, 1920);
    nh.param("height", height, 1080);
    nh.param("fps", fps, 30.0);
    nh.param("trigger_mode", trigger_mode, false);
    nh.param("auto_lock", auto_lock, true);
    nh.param("exposure", exposure_us, 33333);
    nh.param("format", format, std::string("MJPG"));
    nh.param("publish_raw", publish_raw, true);
    nh.param("publish_compressed", publish_compressed, false);
    nh.param("jpeg_quality", jpeg_quality, 80);

    // Initialize camera HID interface
    int hid_fd = findHidDevice();
    if (hid_fd >= 0) {
        // Send OS identification code (required before any other commands)
        if (!sendOSCode(hid_fd)) {
            ROS_ERROR("Failed to send OS identification code");
            close(hid_fd);
            return 1;
        }
        ROS_INFO("Camera OS identification successful");

        // Set camera mode
        if (setTriggerMode(hid_fd, trigger_mode, auto_lock)) {
            ROS_INFO("Camera mode: %s", trigger_mode ? "TRIGGER" : "MASTER");
        } else {
            ROS_WARN("Failed to set camera mode");
        }

        // Set exposure if in trigger mode
        if (trigger_mode) {
            if (setExposure(hid_fd, exposure_us)) {
                ROS_INFO("Exposure locked to %d µs", exposure_us);
            } else {
                ROS_WARN("Failed to set exposure");
            }
        }

        close(hid_fd);
    } else {
        ROS_ERROR("HID device not found");
        return 1;
    }

    // Open camera
    cv::VideoCapture cap(device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_FATAL("Failed to open %s", device.c_str());
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (format == "MJPG") {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }

    // Setup publishers
    ros::Publisher raw_pub, compressed_pub;
    if (publish_raw) {
        raw_pub = nh.advertise<sensor_msgs::Image>(topic, 1);
        ROS_INFO("Publishing raw images on ~/%s", topic.c_str());
    }
    if (publish_compressed) {
        compressed_pub = nh.advertise<sensor_msgs::CompressedImage>(topic + "/compressed", 1);
        ROS_INFO("Publishing compressed images on ~/%s/compressed", topic.c_str());
    }

    std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};

    cv::Mat frame;
    while (ros::ok()) {
      
        if (!cap.read(frame) || frame.empty()) {
            ros::spinOnce();
            continue;
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = frame_id;

        if (publish_raw) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            raw_pub.publish(msg);
        }

        if (publish_compressed) {
            sensor_msgs::CompressedImage compressed_msg;
            compressed_msg.header = header;
            compressed_msg.format = "jpeg";
            cv::imencode(".jpg", frame, compressed_msg.data, jpeg_params);
            compressed_pub.publish(compressed_msg);
        }

        ros::spinOnce();
    }

    return 0;
}
