#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

static int fourccFromString(const std::string& s) {
  if (s.size() != 4) return 0;
  return cv::VideoWriter::fourcc(s[0], s[1], s[2], s[3]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trigger_cam");
  ros::NodeHandle nh("~");

  // Params
  std::string device          = "/dev/video0";
  std::string topic           = "image_raw";
  std::string frame_id        = "camera";
  int width                   = 1280;
  int height                  = 720;
  double fps                  = 15.0;     // nominal; triggers dictate actual arrival
  std::string pixel_format    = "UYVY";   // "UYVY" or "MJPG"
  bool use_gstreamer          = true;     // robust path for UYVY
  int buffer_size             = 1;        // try to keep latency low

  nh.getParam("device", device);
  nh.getParam("topic", topic);
  nh.getParam("frame_id", frame_id);
  nh.getParam("image_width", width);
  nh.getParam("image_height", height);
  nh.getParam("framerate", fps);
  nh.getParam("pixel_format", pixel_format);
  nh.getParam("use_gstreamer", use_gstreamer);
  nh.getParam("buffer_size", buffer_size);

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topic, 1);

  cv::VideoCapture cap;
  bool opened = false;

  if (use_gstreamer) {
    // GStreamer pipeline (handles UYVY → BGR reliably)
    // For UYVY:
    // v4l2src device=/dev/video0 io-mode=mmap !
    // video/x-raw,format=UYVY,width=...,height=...,framerate=15/1 !
    // videoconvert ! video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=1
    std::ostringstream p;
    std::string fmt = (pixel_format == "MJPG" || pixel_format == "mjpg") ? "MJPG" : "UYVY";
    p << "v4l2src device=" << device << " io-mode=mmap ! ";
    if (fmt == "MJPG") {
      // let decoder handle mjpeg
      p << "image/jpeg,width=" << width << ",height=" << height
        << ",framerate=" << (int)std::max(1.0, fps) << "/1 ! jpegdec ! ";
    } else {
      p << "video/x-raw,format=" << fmt
        << ",width=" << width
        << ",height=" << height
        << ",framerate=" << (int)std::max(1.0, fps) << "/1 ! ";
      p << "videoconvert ! ";
    }
    p << "video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=" << std::max(1, buffer_size);

    std::string pipeline = p.str();
    ROS_INFO_STREAM("Opening GStreamer pipeline:\n" << pipeline);
    opened = cap.open(pipeline, cv::CAP_GSTREAMER);
  } else {
    // Plain V4L2 path (may work fine; depends on your OpenCV build)
    ROS_INFO_STREAM("Opening device via V4L2: " << device);
    opened = cap.open(device, cv::CAP_V4L2);
    if (opened) {
      cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
      cap.set(cv::CAP_PROP_FPS,          fps);
      // Try to set FOURCC
      std::string fmt = (pixel_format == "MJPG" || pixel_format == "mjpg") ? "MJPG" : "UYVY";
      cap.set(cv::CAP_PROP_FOURCC, fourccFromString(fmt));
      // Keep latency down
      cap.set(cv::CAP_PROP_BUFFERSIZE, buffer_size);
      // Ask for BGR if supported
      cap.set(cv::CAP_PROP_CONVERT_RGB, 1);
    }
  }

  if (!opened) {
    ROS_FATAL("Failed to open video source.");
    return 1;
  }

  ROS_INFO("trigger_cam started. Publishing on ~/%s", topic.c_str());
  ros::Rate idle(5); // when no subscribers / no frames

  while (ros::ok()) {
    // Only try to grab if someone is listening (optional optimization)
    if (pub.getNumSubscribers() == 0) {
      ros::spinOnce();
      idle.sleep();
      continue;
    }

    cv::Mat bgr;
    bool ok = cap.read(bgr);  // In trigger mode this blocks until a frame arrives
    if (!ok || bgr.empty()) {
      // No frame in time; just loop — in trigger setups, silence is normal between pulses
      ROS_WARN_THROTTLE(5.0, "No frame received (waiting for trigger pulses)...");
      ros::spinOnce();
      continue;
    }

    // Publish
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
    pub.publish(msg);

    ros::spinOnce();
  }

  cap.release();
  return 0;
}
