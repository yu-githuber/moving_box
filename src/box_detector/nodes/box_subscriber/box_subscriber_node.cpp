#include <stdio.h> //my code
#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h> //my code
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

static pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr filter_box_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr whole_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

static sensor_msgs::PointCloud2 cloud_msg;

static ros::Publisher cloud_pub;

using namespace std;

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    return ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("/cloud", 5, cloud_callback);

    ros::spin();
    
    return 0;
}
