#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <linux/hidraw.h>
#include <linux/usbdevice_fs.h>
#include <dirent.h>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <atomic>
#include <string>

static std::atomic<bool> g_shutdown(false);

static void sigintHandler(int) {
    g_shutdown = true;
    ros::requestShutdown();
}

#define BUFFER_LENGTH 65
#define CAMERA_CONTROL_24CUG     0xA8
#define SET_STREAM_MODE_24CUG    0x1C
#define SET_EXPOSURE_24CUG       0x12
#define SET_FLIP_MODE_24CUG      0x0C
#define SET_TO_DEFAULT_24CUG     0xFF
#define FLIP_BOTH_DISABLE        0x00
#define MODE_MASTER              0x00
#define MODE_TRIGGER             0x01
#define SET_SUCCESS              0x01
#define SET_FAIL                 0x00
#define OS_CODE                  0x70
#define LINUX_OS                 0x01
#define ECON_VENDOR_ID           0x2560
#define EXPOSURE_MIN             50
#define EXPOSURE_MAX             100000000
#define HID_TIMEOUT_SEC          5
#define FRAME_TIMEOUT_MS         5000

// Find the USB device node in /dev/bus/usb/ by vendor ID and reset it.
// Returns true if reset succeeded. After reset the camera re-enumerates
// as a fresh device — all firmware state (flip, exposure, mode) is cleared.
static bool usbResetByVendor(uint16_t vendor_id) {
    // Walk /dev/bus/usb/BBB/DDD
    DIR* bus_dir = opendir("/dev/bus/usb");
    if (!bus_dir) {
        ROS_WARN("Cannot open /dev/bus/usb — USB reset unavailable");
        return false;
    }

    struct dirent* bus_entry;
    while ((bus_entry = readdir(bus_dir)) != nullptr) {
        if (bus_entry->d_name[0] == '.') continue;
        std::string bus_path = std::string("/dev/bus/usb/") + bus_entry->d_name;

        DIR* dev_dir = opendir(bus_path.c_str());
        if (!dev_dir) continue;

        struct dirent* dev_entry;
        while ((dev_entry = readdir(dev_dir)) != nullptr) {
            if (dev_entry->d_name[0] == '.') continue;
            std::string dev_path = bus_path + "/" + dev_entry->d_name;

            int fd = open(dev_path.c_str(), O_RDWR);
            if (fd < 0) continue;

            // Read the USB device descriptor — first 18 bytes are standard.
            // idVendor is at offset 8, little-endian uint16.
            unsigned char desc[18] = {0};
            if (read(fd, desc, sizeof(desc)) == sizeof(desc)) {
                uint16_t vid = desc[8] | (desc[9] << 8);
                if (vid == vendor_id) {
                    ROS_INFO("USB reset: found device at %s (VID=0x%04X)", dev_path.c_str(), vid);
                    if (ioctl(fd, USBDEVFS_RESET, 0) < 0) {
                        ROS_WARN("USB reset ioctl failed: %s", strerror(errno));
                        close(fd);
                        closedir(dev_dir);
                        closedir(bus_dir);
                        return false;
                    }
                    close(fd);
                    closedir(dev_dir);
                    closedir(bus_dir);
                    ROS_INFO("USB reset sent — waiting for re-enumeration");
                    usleep(2000000); // 2 s for firmware cold-start + USB re-enum
                    return true;
                }
            }
            close(fd);
        }
        closedir(dev_dir);
    }
    closedir(bus_dir);
    ROS_WARN("USB reset: device with VID=0x%04X not found in /dev/bus/usb", vendor_id);
    return false;
}

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

// Helper: write HID command and wait for response with timeout
static bool hidCmd(int hid_fd, unsigned char* out_buf, unsigned char* in_buf) {
    if (write(hid_fd, out_buf, BUFFER_LENGTH) < 0) return false;

    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(hid_fd, &rfds);
    tv.tv_sec = HID_TIMEOUT_SEC;
    tv.tv_usec = 0;

    if (select(hid_fd + 1, &rfds, NULL, NULL, &tv) <= 0) return false;
    if (read(hid_fd, in_buf, BUFFER_LENGTH) < 0) return false;
    return true;
}

static bool sendOSCode(int hid_fd) {
    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = OS_CODE;
    out_buf[2] = LINUX_OS;

    if (!hidCmd(hid_fd, out_buf, in_buf)) return false;

    // OS code response uses in_buf[2] for status (different from camera control commands)
    return (in_buf[0] == OS_CODE &&
            in_buf[1] == LINUX_OS &&
            in_buf[2] == SET_SUCCESS);
}

static bool resetToDefault(int hid_fd) {
    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = CAMERA_CONTROL_24CUG;
    out_buf[2] = SET_TO_DEFAULT_24CUG;

    if (!hidCmd(hid_fd, out_buf, in_buf)) return false;

    return (in_buf[0] == CAMERA_CONTROL_24CUG &&
            in_buf[1] == SET_TO_DEFAULT_24CUG &&
            in_buf[6] == SET_SUCCESS);
}

static unsigned char getFlip(int hid_fd) {
    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = CAMERA_CONTROL_24CUG;
    out_buf[2] = 0x0B; // GET_FLIP_MODE_24CUG

    if (!hidCmd(hid_fd, out_buf, in_buf)) return 0xFF; // 0xFF = error sentinel
    if (in_buf[0] == CAMERA_CONTROL_24CUG &&
        in_buf[1] == 0x0B &&
        in_buf[6] == SET_SUCCESS)
        return in_buf[2]; // actual flip value
    return 0xFF;
}

static bool setFlip(int hid_fd, unsigned char flip_value) {
    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = CAMERA_CONTROL_24CUG;
    out_buf[2] = SET_FLIP_MODE_24CUG;
    out_buf[3] = flip_value;

    if (!hidCmd(hid_fd, out_buf, in_buf)) return false;

    return (in_buf[0] == CAMERA_CONTROL_24CUG &&
            in_buf[1] == SET_FLIP_MODE_24CUG &&
            in_buf[6] == SET_SUCCESS);
}

static bool setExposure(int hid_fd, unsigned int exposure_us) {
    if (exposure_us < EXPOSURE_MIN || exposure_us > EXPOSURE_MAX) return false;

    unsigned char out_buf[BUFFER_LENGTH] = {0};
    unsigned char in_buf[BUFFER_LENGTH] = {0};

    out_buf[1] = CAMERA_CONTROL_24CUG;
    out_buf[2] = SET_EXPOSURE_24CUG;
    out_buf[3] = (exposure_us >> 24) & 0xFF;
    out_buf[4] = (exposure_us >> 16) & 0xFF;
    out_buf[5] = (exposure_us >> 8)  & 0xFF;
    out_buf[6] =  exposure_us        & 0xFF;

    if (!hidCmd(hid_fd, out_buf, in_buf)) return false;

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

    if (!hidCmd(hid_fd, out_buf, in_buf)) return false;

    return (in_buf[0] == CAMERA_CONTROL_24CUG &&
            in_buf[1] == SET_STREAM_MODE_24CUG &&
            in_buf[6] == SET_SUCCESS);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trigger_cam", ros::init_options::NoSigintHandler);
    signal(SIGINT, sigintHandler);
    ros::NodeHandle nh("~");

    std::string device, topic, frame_id, format;
    int width, height, jpeg_quality, exposure_us;
    double fps;
    bool trigger_mode, auto_lock, publish_raw, publish_compressed;

    nh.param("device",             device,             std::string("/dev/video0"));
    nh.param("topic",              topic,              std::string("image_raw"));
    nh.param("frame_id",           frame_id,           std::string("camera"));
    nh.param("width",              width,              1920);
    nh.param("height",             height,             1080);
    nh.param("fps",                fps,                30.0);
    nh.param("trigger_mode",       trigger_mode,       false);
    nh.param("auto_lock",          auto_lock,          true);
    nh.param("exposure",           exposure_us,        33333);
    nh.param("format",             format,             std::string("MJPG"));
    nh.param("publish_raw",        publish_raw,        true);
    nh.param("publish_compressed", publish_compressed, false);
    nh.param("jpeg_quality",       jpeg_quality,       80);

    // --- USB power cycle ---
    // Reset the camera at the USB level before any HID/V4L2 interaction.
    // This clears all firmware state (flip, exposure, mode) left from previous
    // sessions, eliminating the need to rely on resetToDefault being reliable.
    if (!usbResetByVendor(ECON_VENDOR_ID)) {
        ROS_WARN("USB reset failed or unavailable — proceeding without power cycle");
    }

    // --- HID initialization ---
    int hid_fd = findHidDevice();
    if (hid_fd < 0) {
        ROS_ERROR("HID device not found after USB reset");
        return 1;
    }

    // 1. OS identification (required before any other HID command)
    if (!sendOSCode(hid_fd)) {
        ROS_ERROR("Failed to send OS identification code");
        close(hid_fd);
        return 1;
    }
    ROS_INFO("Camera OS identification successful");
    usleep(100000);

    // Helper lambda for flip disable with readback retry
    auto enforceFlipDisabled = [&]() -> bool {
        const int MAX_FLIP_RETRIES = 5;
        for (int i = 0; i < MAX_FLIP_RETRIES; i++) {
            if (i > 0) usleep(150000);
            if (!setFlip(hid_fd, FLIP_BOTH_DISABLE)) {
                ROS_WARN("setFlip attempt %d/%d: command failed", i + 1, MAX_FLIP_RETRIES);
                continue;
            }
            usleep(100000);
            unsigned char actual = getFlip(hid_fd);
            if (actual == FLIP_BOTH_DISABLE) {
                ROS_INFO("Flip disabled (confirmed after %d attempt(s))", i + 1);
                return true;
            }
            ROS_WARN("setFlip attempt %d/%d: readback=0x%02X, retrying",
                     i + 1, MAX_FLIP_RETRIES, actual);
        }
        return false;
    };

    // 2. Set exposure while still in master mode (before the auto-function lock)
    if (setExposure(hid_fd, exposure_us)) {
        ROS_INFO("Exposure set to %d us", exposure_us);
    } else {
        ROS_WARN("Failed to set exposure");
    }
    usleep(100000);

    // 3. Switch mode — auto_lock freezes the exposure value set above
    if (setTriggerMode(hid_fd, trigger_mode, auto_lock)) {
        ROS_INFO("Camera mode: %s (auto_lock=%s)",
                 trigger_mode ? "TRIGGER" : "MASTER",
                 auto_lock ? "true" : "false");
    } else {
        ROS_WARN("Failed to set camera mode");
    }
    usleep(100000);

    // 4. Set flip AFTER mode switch — setTriggerMode can disturb flip state in firmware.
    //    This is the last HID command so nothing can override it afterwards.
    if (!enforceFlipDisabled()) {
        ROS_ERROR("Failed to disable flip after %d attempts — aborting", 5);
        close(hid_fd);
        return 1;
    }
    usleep(100000);

    close(hid_fd);

    // Wait for camera firmware to settle after HID commands before opening V4L2.
    // Exposure changes alter sensor timing — opening too early causes V4L2 to negotiate
    // buffer geometry while the camera is still reconfiguring, leading to stretch/warp.
    usleep(500000); // 500 ms

    // --- Open V4L2 device ---
    cv::VideoCapture cap(device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_FATAL("Failed to open %s", device.c_str());
        return 1;
    }

    // Set FOURCC before resolution — V4L2 uses the pixel format to determine
    // valid resolution ranges, so wrong order can cause silent fallback to a
    // different resolution with wrong buffer stride (produces stretched images).
    if (format == "MJPG") {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS,          fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE,   1);

    // Read back actual negotiated parameters.
    {
        int actual_w      = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_h      = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        int actual_fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
        char fourcc_str[5] = {
            static_cast<char>((actual_fourcc >>  0) & 0xFF),
            static_cast<char>((actual_fourcc >>  8) & 0xFF),
            static_cast<char>((actual_fourcc >> 16) & 0xFF),
            static_cast<char>((actual_fourcc >> 24) & 0xFF),
            '\0'
        };

        if (actual_w != width || actual_h != height) {
            ROS_FATAL("Requested %dx%d but camera negotiated %dx%d — check format/resolution params",
                      width, height, actual_w, actual_h);
            return 1;
        }
        ROS_INFO("Resolution confirmed: %dx%d", actual_w, actual_h);

        if (format != std::string(fourcc_str)) {
            ROS_WARN("Requested format %s but camera negotiated %.4s",
                     format.c_str(), fourcc_str);
        } else {
            ROS_INFO("Format confirmed: %.4s", fourcc_str);
        }
    }

    // Open a separate fd on the device for select()-based frame timeout.
    // CAP_PROP_VIDEO_STREAM requires OpenCV >= 4.5.1; we are on 4.5.0 so open manually.
    int v4l2_fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (v4l2_fd < 0) {
        ROS_WARN("Could not open %s for polling (%s); cap.read() may block in trigger mode",
                 device.c_str(), strerror(errno));
    }

    // --- Publishers ---
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

    // In trigger mode, discard the first frame after startup — it may be a stale
    // buffer captured before the new exposure setting took effect, causing warp/stretch.
    bool discard_first = trigger_mode;

    // --- Main capture loop ---
    cv::Mat frame;
    while (ros::ok() && !g_shutdown) {
        // Gate cap.read() with select() so we never block forever waiting for a trigger
        if (v4l2_fd >= 0) {
            fd_set rfds;
            struct timeval tv;
            FD_ZERO(&rfds);
            FD_SET(v4l2_fd, &rfds);
            tv.tv_sec  = FRAME_TIMEOUT_MS / 1000;
            tv.tv_usec = (FRAME_TIMEOUT_MS % 1000) * 1000;

            int ret = select(v4l2_fd + 1, &rfds, NULL, NULL, &tv);
            if (ret < 0) {
                if (errno == EINTR) break; // SIGINT received, exit immediately
                ROS_ERROR("select() on camera fd failed: %s", strerror(errno));
                break;
            }
            if (ret == 0) {
                ROS_DEBUG_THROTTLE(10.0, "No frame within %d ms (trigger_mode=%s)",
                                   FRAME_TIMEOUT_MS, trigger_mode ? "true" : "false");
                ros::spinOnce();
                continue;
            }
        }

        if (!cap.read(frame) || frame.empty()) {
            ros::spinOnce();
            continue;
        }

        if (discard_first) {
            discard_first = false;
            ROS_INFO("Discarded first frame (startup flush)");
            ros::spinOnce();
            continue;
        }

        std_msgs::Header header;
        header.stamp    = ros::Time::now();
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

    if (v4l2_fd >= 0) close(v4l2_fd);
    cap.release();
    return 0;
}
