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

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

static pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr whole_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());

static sensor_msgs::PointCloud2 cloud_msg;

static ros::Publisher cloud_pub;

using namespace std;

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    // Read in the cloud data
    pcl::fromROSMsg(*input,*whole_cloud);
 
	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(300);
	seg.setDistanceThreshold(0.05);
    
    int i=0;
	while (whole_cloud->points.size() > 0)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(whole_cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		
		char filepath2[100];
		sprintf(filepath2, "plane_%d.pcd", i);
		
 
		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(whole_cloud);
		extract.setIndices(inliers);
 
 
		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
 
		if (cloud_plane->points.size()>0)
		{
			std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
 
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
			colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
			std::vector<unsigned int> colors;
			colors.push_back(static_cast<unsigned int> (rand() % 256));
			colors.push_back(static_cast<unsigned int> (rand() % 256));
			colors.push_back(static_cast<unsigned int> (rand() % 256));
 
			colored_cloud->width = cloud_plane->width;
			colored_cloud->height = cloud_plane->height;
 
			colored_cloud->is_dense = cloud_plane->is_dense;
			for (size_t i_point = 0; i_point < cloud_plane->points.size(); i_point++)
			{
				pcl::PointXYZRGB point;
				point.x = *(cloud_plane->points[i_point].data);
				point.y = *(cloud_plane->points[i_point].data + 1);
				point.z = *(cloud_plane->points[i_point].data + 2);
				point.r = colors[0];
				point.g = colors[1];
				point.b = colors[2];
				colored_cloud->points.push_back(point);
			}
			pcl::io::savePCDFileASCII(filepath2, *colored_cloud);
			++i;
		}
		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*whole_cloud = *cloud_f;
	}
 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("/cloud", 5, cloud_callback);

    ros::spin();
    
    return 0;
}

	