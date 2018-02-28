#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "boost/foreach.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
typedef pcl::PointCloud<pcl::PointXY> PointCloud;

void getData (const pcl::PointCloud2 cloud_msg) {
	printf ("Cloud: width = %d, height = %d\n", cloud_msg->width, cloud_msg->height);
	BOOST_FOREACH (const pcl::PointXY& point, cloud_msg->points) {
		printf("\t(%f, %f)\n", point.x, point.y);
	}
    //sensor_msgs::PointCloud2 cloud;
    //cloud.header = pcl_conversions::toPCL (cloud_msg.header);
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "collect");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<PointCloud>("cloud", 10, getData);
	ros::spin();
}