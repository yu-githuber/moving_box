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
#include <pcl/filters/crop_hull.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

static pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr up_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr trans_up_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr filter_box_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr whole_cloud(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

static sensor_msgs::PointCloud2 cloud_msg;

static ros::Publisher cloud_pub;

static Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_publisher");
    ros::NodeHandle nh;

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 5);

    pcl::PointXYZ p_plane, p_box;

    //generate plane_cloud
    for (int i = 0; i < 101; i++)
    {
        for (int j = 0; j < 101; j++)
        {
            p_plane.x = 0.1 * i - 5;
            p_plane.y = 0.1 * j - 5;
            p_plane.z = 0;
            plane_cloud->push_back(p_plane);
        }
    }

    //generate box_cloud (method one)
    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            for (int k = 0; k < 11; k++)
            {
                p_box.x = 0.1 * i - 0.5;
                p_box.y = 0.1 * j - 0.5;
                p_box.z = 0.1 * k;
                box_cloud->push_back(p_box);
            }
        }
    }

    pcl::CropBox<pcl::PointXYZ> box_filter(true);
    box_filter.setMin(Eigen::Vector4f(-0.48, -0.48, 0.02, 1.0));
    box_filter.setMax(Eigen::Vector4f(0.48, 0.48, 0.98, 1.0));
    box_filter.setNegative(true);
    box_filter.setInputCloud(box_cloud);
    box_filter.filter(*filter_box_cloud);

    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            p_box.x = 0.1 * i - 0.5;
            p_box.y = 0.1 * j - 0.5;
            p_box.z = 1;
            up_cloud->push_back(p_box);
        }
    }

    /*
    //generate box_cloud (method two)
    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            p_box.x = 0.1 * i - 0.5;
            p_box.y = 0.1 * j - 0.5;
            p_box.z = 0;
            box_cloud->push_back(p_box);
        }
    }

    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            p_box.x = 0.1 * i - 0.5;
            p_box.y = 0.1 * j - 0.5;
            p_box.z = 1;
            box_cloud->push_back(p_box);
        }
    }

    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            p_box.x = -0.5;
            p_box.y = 0.1 * i - 0.5;
            p_box.z = 0.1 * j;
            box_cloud->push_back(p_box);
        }
    }

    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            p_box.x = 0.5;
            p_box.y = 0.1 * i - 0.5;
            p_box.z = 0.1 * j;
            box_cloud->push_back(p_box);
        }
    }

    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            p_box.x = 0.1 * i - 0.5;
            p_box.y = -0.5;
            p_box.z = 0.1 * j;
            box_cloud->push_back(p_box);
        }
    }

    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            p_box.x = 0.1 * i - 0.5;
            p_box.y = 0.5;
            p_box.z = 0.1 * j;
            box_cloud->push_back(p_box);
        }
    }*/

    *transform_cloud = *filter_box_cloud;
    *trans_up_cloud = *up_cloud;

    ros::Rate ros_rate(5);

    int count = 0;

    while (ros::ok())
    {
        if (count == 4)
        {
            float _tf_x = 0.1 * (rand() % 3 - 1);
            float _tf_y = 0.1 * (rand() % 3 - 1);
            float _tf_yaw = (rand() % 90) * M_PI / 180;
            Eigen::Translation3f tl(_tf_x, _tf_y, 0);
            Eigen::AngleAxisf rot_x(0, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf rot_y(0, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf rot_z(_tf_yaw, Eigen::Vector3f::UnitZ());
            trans = (tl * rot_z * rot_y * rot_x).matrix();
            pcl::transformPointCloud(*filter_box_cloud, *transform_cloud, trans);
            pcl::transformPointCloud(*up_cloud, *trans_up_cloud, trans);

            count = 0;
        }
        else
        {
            count++;
        }

        *whole_cloud = *plane_cloud + *transform_cloud;

        
        //remove bottom cloud (method one)
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_hull_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        vector<pcl::Vertices> vertices;
        pcl::Vertices vt;

        hull_cloud->push_back(pcl::PointXYZ(-0.48,-0.48,0));
        hull_cloud->push_back(pcl::PointXYZ(0.48,-0.48,0));
        hull_cloud->push_back(pcl::PointXYZ(0.48,0.48,0));
        hull_cloud->push_back(pcl::PointXYZ(-0.48,0.48,0));
        pcl::transformPointCloud(*hull_cloud, *trans_hull_cloud, trans);
        vt.vertices.push_back(0);
        vt.vertices.push_back(1);
	    vt.vertices.push_back(2);
	    vt.vertices.push_back(3);
        vertices.push_back(vt);

        pcl::CropHull<pcl::PointXYZ> square_hull;
        square_hull.setHullIndices(vertices);
        square_hull.setHullCloud(trans_hull_cloud);
        square_hull.setInputCloud(whole_cloud);
        square_hull.setCropOutside(false);
        square_hull.setDim(2);
        square_hull.filter(*output_cloud);

        *output_cloud = *output_cloud + *trans_up_cloud;

        /*
        //remove bottom cloud (method two)
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr trans_crop_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        //pcl::PointXYZI min_pt(-0.48, -0.48, -0.02, 1.0);
        //pcl::PointXYZI max_pt(0.48, 0.48, 0.02, 1.0);
        pcl::PointXYZI min_pt, max_pt;
        min_pt.x = -0.48;
        min_pt.y = -0.48;
        min_pt.z = -0.02;
        min_pt.intensity = 1;
        max_pt.x = 0.48;
        max_pt.y = 0.48;
        max_pt.z = 0.02;
        max_pt.intensity = 1;
        crop_cloud->points.push_back(min_pt);
        crop_cloud->points.push_back(max_pt);
        
        pcl::transformPointCloud(*crop_cloud, *trans_crop_cloud, trans);
        cout << trans << endl;
        cout << trans_crop_cloud->points[0] << endl;
        cout << trans_crop_cloud->points[1] << endl;
        cout << "++++++++++++++++++++" << endl;

        pcl::CropBox<pcl::PointXYZ> crop_filter(true);
        crop_filter.setMin(Eigen::Vector4f(trans_crop_cloud->points[0].x, trans_crop_cloud->points[0].y, trans_crop_cloud->points[0].z, 1.0));
        crop_filter.setMax(Eigen::Vector4f(trans_crop_cloud->points[1].x, trans_crop_cloud->points[1].y, trans_crop_cloud->points[1].z, 1.0));
        crop_filter.setNegative(true);
        crop_filter.setInputCloud(whole_cloud);
        crop_filter.filter(*output_cloud);*/

        for (int i = 0; i < (static_cast<int>(output_cloud->size())); i++)
        {
            int noise = rand() % 5 - 2;
            output_cloud->points[i].x = output_cloud->points[i].x + 0.0001 * noise;
            output_cloud->points[i].y = output_cloud->points[i].y + 0.0001 * noise;
            output_cloud->points[i].z = output_cloud->points[i].z + 0.0001 * noise;
        }

        pcl::toROSMsg(*output_cloud, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);

        ros_rate.sleep();
    }

    return 0;
}
