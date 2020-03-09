#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <sensor_msgs/Imu.h>

#include "Thirdparty/DLib/FileFunctions.h"

DEFINE_string(data_path, "", "path to datas");
DEFINE_string(filename, "/bag.bag", "bag file name, will be put into data_path");
DEFINE_string(data_ext, ".png", "image format, with dot like .png");
DEFINE_string(left_image_topic, "/cam0/image_raw", "image topic");
DEFINE_string(right_image_topic, "/cam1/image_raw", "image topic");
DEFINE_string(imu_topic, "/imu_raw", "imu topic");

using namespace std;

unsigned long long get_time_from_string(std::string &name) {
  int pos = name.find(".png");
  pos--;
  std::string split;
  while (pos >= 0) {
    if (name[pos] >= '0' && name[pos] <= '9') {
      split.insert(split.begin(), name[pos]);
    } else {
      break;
    }
    pos--;
  }
  unsigned long long t = stoull(split);
  return t;
};

void write_image_into_bag(
    const std::string &path, rosbag::Bag &bag, const std::string &topic) {
  // Vector of paths to image
  vector<string> filenames =
      DUtils::FileFunctions::Dir(path.c_str(), FLAGS_data_ext.c_str(), true);
  LOG(ERROR) << "Images: " << filenames.size() << " path " << path;

  unsigned long long ts;
  double second_t;

  for (size_t i = 0; i < filenames.size(); i++) {
    if (!ros::ok())
      break;

    cv::Mat im = cv::imread(filenames[i], cv::IMREAD_GRAYSCALE);
    if (!im.empty()) {
      cv_bridge::CvImage cvImage;
      cvImage.image = im;
      cvImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      ///this part should be the same with image read from file, I set it to IMREAD_GRAYSCALE

      ts = get_time_from_string(filenames[i]);
      second_t = ts /
                 1000000000.0; //in my system, time is ns, so make it /1000000000
      cvImage.header.stamp = ros::Time(second_t);
      bag.write(topic.c_str(),
                ros::Time(second_t), cvImage.toImageMsg());
//      LOG(ERROR) << i << " / " << filenames.size() << endl;
    } else {
      LOG(ERROR) << "image " << filenames[i] << " read fail";
    }
  }
}

void write_imu_into_bag(
    const std::string &path, rosbag::Bag &bag, const std::string &topic){
  std::ifstream imu_data(path);

  if (imu_data.is_open()) {
    const int imu_size = 6;
    std::vector<double> imu(imu_size);
    char comma;
    std::stringstream ss;
    std::string line;
    unsigned long long ts;
    double second_t;
    sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
    while (!imu_data.eof()) {
      std::getline(imu_data, line);
      if (line.empty()) {
        break;
      }
      ss.clear();
      ss.str(line);
      ss >> ts;
      ss >> comma;
      CHECK(comma == ',');
      for (int i = 0; i < imu_size; i++) {
        ss >> imu[i];
        ss >> comma;
        CHECK(comma == ',');
      }
      second_t = ts /
                 1000000000.0; //in my system, time is ns, so make it /1000000000
      imu_msg->header.stamp = ros::Time(second_t);
      imu_msg->angular_velocity.x = imu[0];
      imu_msg->angular_velocity.y = imu[1];
      imu_msg->angular_velocity.z = imu[2];
      imu_msg->linear_acceleration.x = imu[3];
      imu_msg->linear_acceleration.y = imu[4];
      imu_msg->linear_acceleration.z = imu[5];
      if (!ros::ok())
        break;
      bag.write(topic.c_str(),
          ros::Time(second_t), imu_msg);
    }
    imu_data.close();
  } else {
    LOG(ERROR) << "no imu data in " << path;
  }
}

int main(int argc, char **argv) {
  google::SetVersionString("1.0.0");
  google::SetUsageMessage(std::string(argv[0]) + " [OPTION]");
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]); // option --[also]logtostderr
  //        --stderrthreshold=0
  ros::init(argc, argv, "BagFromImages");

  CHECK(!FLAGS_data_path.empty())
      << "Usage: rosrun BagFromImages BagFromImages"
        " <path to image directory> <image extension .ext> "
        "<frequency> <path to output bag>";

  std::string image_path[2] = {FLAGS_data_path + "/cam0/data/", FLAGS_data_path + "/cam1/data/"};

  ros::start();
  // Output bag
  std::string bagname = FLAGS_data_path + FLAGS_filename;
  rosbag::Bag bag_out(bagname.c_str(), rosbag::bagmode::Write);

  if (!FLAGS_left_image_topic.empty()) {
    write_image_into_bag(image_path[0], bag_out, FLAGS_left_image_topic);
  }

  if (!FLAGS_right_image_topic.empty()) {
    write_image_into_bag(image_path[1], bag_out, FLAGS_right_image_topic);
  }

  if (!FLAGS_imu_topic.empty()) {
    std::string imu_file = FLAGS_data_path + "/imu0/data.csv";
    write_imu_into_bag(imu_file, bag_out, FLAGS_imu_topic);
  }

  bag_out.close();
  ros::shutdown();
  LOG(ERROR) << "finish write";
  return 0;
}
