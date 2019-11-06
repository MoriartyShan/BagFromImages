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

#include "Thirdparty/DLib/FileFunctions.h"


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

int main(int argc, char **argv) {
  ros::init(argc, argv, "BagFromImages");

  if (argc != 5) {
    LOG(FATAL) << "Usage: rosrun BagFromImages BagFromImages"
                  " <path to image directory> <image extension .ext> "
                  "<frequency> <path to output bag>";
    return 0;
  }

  ros::start();

  // Vector of paths to image
  vector<string> filenames =
      DUtils::FileFunctions::Dir(argv[1], argv[2], true);

  LOG(ERROR) << "Images: " << filenames.size() << endl;
  // Output bag
  rosbag::Bag bag_out(argv[4], rosbag::bagmode::Write);
  double t;
  for (size_t i = 0; i < filenames.size(); i++) {
    if (!ros::ok())
      break;

    cv::Mat im = cv::imread(filenames[i], cv::IMREAD_GRAYSCALE);
    cv_bridge::CvImage cvImage;
    cvImage.image = im;
    cvImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    ///this part should be the same with image read from file, I set it to IMREAD_GRAYSCALE

    unsigned long long this_time = get_time_from_string(filenames[i]);
    t = this_time /
        1000000000.0; //in my system, time is ns, so make it /1000000000
    cvImage.header.stamp = ros::Time(t);
    bag_out.write("/camera/image_raw",ros::Time(t),cvImage.toImageMsg());
    LOG(ERROR) << i << " / " << filenames.size() << endl;
  }

  bag_out.close();
  ros::shutdown();
  return 0;
}
