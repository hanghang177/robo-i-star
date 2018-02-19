// #include <stdio.h>
// #include "ros/ros.h"
// #include <iostream>
// #include "sensor_msgs/LaserScan.h"
// #include "pcl_ros/point_cloud.h"
// #include "pcl/point_types.h"
// #include "pcl/filters/passthrough.h"
// #include "pcl/filters/statistical_outlier_removal.h"

// ros::Publisher pub;
// int frameCount = 0;
// typedef pcl::PointCloud<pcl::PointXY> PointCloud;
// void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
// 	PointCloud::Ptr cloud (new PointCloud);
// 	cloud->header.frame_id = "Frame: " + frameCount;
// 	frameCount++;
//     int count = scan->scan_time / scan->time_increment;
//     cloud->width = count;
//     cloud->height = 1;
//     //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
//     //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
//     int actualSize = 0;
//     for(int i = 0; i < count; i++) {
//     	float radians = scan->angle_min + scan->angle_increment * i;
//     	float range = scan->ranges[i];
//     	if (!(range > scan->range_max) && !(range < scan->range_min)) {
//     		double x = range * cos(radians);
//     		double y = range * sin(radians);
//     		pcl::PointXY point;
//     		point.x = x;
//     		point.y = y;
//     		cloud->points.push_back (point);
//     		actualSize++;
//     	}
//     }
//     cloud->width = actualSize;
//     //Declare new cloud for after filtering
//     pcl::PassThrough<pcl::PointXY> pass_x;
//     pcl::PassThrough<pcl::PointXY> pass_y;

//     pass_x.setInputCloud (cloud);
//     //Filter the cloud
//     // pcl::StatisticalOutlierRemoval<pcl::PointXY> sor;
//     // sor.setInputCloud (cloud);
//     // sor.setMeanK (50);
//     // sor.setStddevMulThresh (1.0);
//     // sor.filter (*filtered);
//     //cloud->header.stamp = ros::Time::now().toNSec();
//     pub.publish (cloud);

// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "obstacleAvoidance");
//     ros::NodeHandle n;

//     ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

//     pub = n.advertise<PointCloud> ("cloud", 10);

//     ros::spin();

//     return 0;
// }
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXY  Point2D;
typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud<Point2D> PointCloud2D;
typedef pcl::PointCloud<Point3D> PointCloud3D;

ros::Publisher pub;
int frameCount = 0;
float range_min = 0;
float range_max = 10000;
inline void PointCloud3Dto2D(PointCloud3D& in, PointCloud2D& out)
{
  out.width = in.width;
  out.height = in.height;
  for (size_t i = 0; i < in.points.size(); i++)
  {
      Point2D p;
      p.x = in.points[i].x;
      p.y = in.points[i].y;
      out.points.push_back(p);
  }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;

    PointCloud3D::Ptr cloud3D(new PointCloud3D);
    cloud3D->header.frame_id = "Frame: " + (frameCount++);
    cloud3D->height = 1;

    for (int i = 0; i < count; i++)
    {
        float radians = scan->angle_min + scan->angle_increment * i;
        float range = scan->ranges[i];

        if ((range > scan->range_max) || (range < scan->range_min)) continue;
        if ((range < range_min) || (range > range_max)) continue;
        Point3D point3D;
        point3D.x = range * cos(radians);
        point3D.y = range * sin(radians);
        point3D.z = 0.0;
        cloud3D->points.push_back(point3D);
    }
    cloud3D->width = cloud3D->points.size();

    // Declare new cloud for filtering
    PointCloud3D::Ptr cloud3D_filtered(new PointCloud3D);

    // pcl::PassThrough<Point3D> pass_x;
    // pass_x.setInputCloud(cloud3D);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(0.0, 1.0);
    // pass_x.filter(*cloud3D_filtered);

    pcl::StatisticalOutlierRemoval<Point3D> sor;
    sor.setInputCloud(cloud3D);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filter(*cloud3D_filtered);
    cloud3D->header.stamp = ros::Time::now().toNSec();

    PointCloud2D::Ptr cloud(new PointCloud2D);
    PointCloud3Dto2D(*cloud3D, *cloud);
    ROS_INFO ("Starting");
    for (int i = 0; i < cloud->points.size(); i++) {
    	ROS_INFO ("(%f, %f)\n", cloud->points[i].x, cloud->points[i].y);
	}

    pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacleAvoidance");
    ros::NodeHandle n;

    pub = n.advertise<PointCloud2D>("cloud", 10);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}
