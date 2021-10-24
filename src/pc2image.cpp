#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>
#include <opencv2/core/core.hpp>
#include <math.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher imgD_pub;
boost::shared_ptr<pcl::RangeImageSpherical> rngSpheric;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

float maxlen =20;
float minlen = 0.1;
float angular_resolution_x = 0.25f;
float angular_resolution_y = 0.85f;
float max_angle_width= 360.0f;
float max_angle_height = 360.0f;
//string poinCloud_topic = "/velodyne_points";


void callback(const PointCloud::ConstPtr& msg_pointCloud)
{
  if (msg_pointCloud == NULL) return;


  rngSpheric->pcl::RangeImage::createFromPointCloud(*msg_pointCloud, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_height), pcl::deg2rad(max_angle_height),
                                       Eigen::Affine3f::Identity(), coordinate_frame, 0.0f, 0.0f, 0);

  rngSpheric->header.frame_id = msg_pointCloud->header.frame_id;
  rngSpheric->header.stamp    = msg_pointCloud->header.stamp;

  int cols = rngSpheric->width;
  int rows = rngSpheric->height;
  cv::Mat _rangeImage = cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));;
  float range, r;

    for (int i=0; i<cols; ++i)    
      for (int j=0; j<rows; ++j)
      {
        float r = rngSpheric->getPoint(i, j).range;
        if(!std::isinf(r))
            range = (65536.0 / (maxlen - minlen))*(r-minlen);
        else
            range = 0;
        if (range>65536)
                range = 65536;
        _rangeImage.at<ushort>(j, i) = 1-range;
      }

   sensor_msgs::ImagePtr image_msg;
   image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", _rangeImage).toImageMsg();
   imgD_pub.publish(image_msg);

}

int main(int argc, char** argv)
{


  ros::init(argc, argv, "pontCloud2dephtImage");
  ros::NodeHandle nh;  

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/angular_resolution_x", angular_resolution_x);
  nh.getParam("/angular_resolution_y", angular_resolution_y);
  nh.getParam("/max_angle_width", max_angle_width);
  nh.getParam("/max_angle_height", max_angle_height);

  ros::Subscriber sub = nh.subscribe<PointCloud>("/velodyne_points", 10, callback);
  rngSpheric = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
  imgD_pub = nh.advertise<sensor_msgs::Image>("/depht_image", 10);
  ros::spin();
  return 0;
}
