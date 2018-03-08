#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sstream>
#include <string>
#include <std_msgs/String.h>
//#include "lidar/avoid.h"
typedef pcl::PointXY  Point2D;
typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud<Point2D> PointCloud2D;
typedef pcl::PointCloud<Point3D> PointCloud3D;
//The range at which all objects should keep away from the robot
float robotRadius = 5;
ros::Publisher pub;
//How many frames have been received and sent
int frameCount = 0;
//The numbers to filter out bad values
float range_min = 0;
float range_max = 10;
//The amout the robot reacts when it finds an obstacle
float xGain = 100, yGain = 10;
//How sharp of a turn it will be when the robot finds an object
float turnSharpness = 40;
float mag (float value) {
  if (value < 0) {
      return -value;
  } else {
    return value;
  }
}
class Vector {
public:
	float x, y;
	Vector (float xx, float yy) {
		x = xx;
		y = yy;
	}
};
//A class to represent a wall as a line
class Line {
public:
	float a, b, c;
	PointCloud3D::Ptr line_points;
	Line (pcl::ModelCoefficients::Ptr coef, PointCloud3D::Ptr cloud) {
		line_points = cloud;
		a = coef->values[0];
		b = coef->values[1];
		//Ignore z component of coefficients
		c = coef->values[3];
	}
};
//Class to represent all objects as a simple circle cluster
class Circle {
public:
	float radius;
	float centerX, centerY;
	Circle () {
		radius = 0;
		centerX = 0;
		centerY = 0;
	}
};
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
    std::cout << "Incoming scan with " << cloud3D->width << " data points\n";
    // Declare new cloud for filtering
    PointCloud3D::Ptr cloud3D_filtered(new PointCloud3D);

    //std::cout << "Original scan size: " << cloud3D->points.size() << "\n";

    // pcl::PassThrough<Point3D> pass_x;
    // pass_x.setInputCloud(cloud3D);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(0.0, 1.0);
    // pass_x.filter(*cloud3D_filtered);
    //Statistical outlier filter
    pcl::StatisticalOutlierRemoval<Point3D> sor;
    sor.setInputCloud(cloud3D);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filter(*cloud3D_filtered);
    std::cout << "Filtered cloud using sor now has " << cloud3D->points.size() << " data points\n";
    //Downsample data maybe?



    //Segmentation object
    pcl::SACSegmentation<Point3D> seg;
    PointCloud3D::Ptr cloud_f (new PointCloud3D);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    PointCloud3D::Ptr cloud_plane (new PointCloud3D);
    //Cloud for storing coefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    //const Eigen::Vector3f ax(0,0,1);
    //seg.setAxis(ax);
    //Possibly may need to adjust this?
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (10);
    //std::vector<PointCloud3D::Ptr> inliers_group;
    //std::vector<pcl::ModelCoefficients::Ptr> coefficients_group;
    std::vector<Line> walls;
     int i = 0, nr_points = (int) cloud3D_filtered->points.size ();
     while (cloud3D_filtered->points.size () > .3 * nr_points) {
    	seg.setInputCloud (cloud3D_filtered);
    	seg.segment (*inliers, *coefficients);
    	// for (int i = 0; i < coefficients->values.size(); i++) {
    	// 	std::cout << (i + 1) << ": " << coefficients->values[i] << "\n";
    	// }
    	//If it's 0 then there aren't any point that are not outliers
    	if (inliers->indices.size () == 0) {
    		std::cout << "Couldn't estimate a planar model for the given dataset.\n";
    		break;
    	}

    	//Extract the planar inliers
    	pcl::ExtractIndices<Point3D> extract;
    	extract.setInputCloud (cloud3D_filtered);
    	extract.setIndices (inliers);
    	extract.setNegative (false);
    	std::cout << "Inliers size: " << inliers->indices.size() << "\n";
    	//Get the points associated with the planar surface
    	extract.filter (*cloud_plane);
    	//inliers_group.push_back(inliers);
    	//coefficients_group.push_back(coefficients);
    	Line line (coefficients, cloud_plane);
    	walls.push_back(line);
    	//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points\n";
    	std::cout << "Coefficients are:\n a = " << coefficients->values[0] << ", b = " << coefficients->values[1] << ", c = " << coefficients->values[3] << "\n";
    	//Remove the planar inliers, extract the rest.
    	 extract.setNegative (true);
    	 extract.filter (*cloud_f);
    	 *cloud3D_filtered = *cloud_f;
     }
     std::cout << "There were a total of " << walls.size() << " walls\n";
    //std::cout << "The number of coefficients is: " << coefficients_group.size() << "\n";

 //    float model_ss_ (0.04f);
	// float scene_ss_ (0.04f);
	// float rf_rad_ (0.2f);
	// float descr_rad_ (1.8f);
	// float cg_size_ (0.4f);
	// float cg_thresh_ (-0.5f);
    //Creating the Kd tree for search method of extraction
    pcl::search::KdTree<Point3D>::Ptr tree (new pcl::search::KdTree<Point3D>);
    tree->setInputCloud (cloud_plane);
    //Vector to store all the clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point3D> ec;
    ec.setClusterTolerance (50);
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud3D_filtered);
    ec.extract (cluster_indices);
    std::cout << "Cluster indices size: " << cluster_indices.size() << "\n";
    cloud3D->header.stamp = ros::Time::now().toNSec();
    std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > clusters;
    //PointCloud2D::Ptr cloud(new PointCloud2D);
    //PointCloud3Dto2D(*cloud3D_filtered, *cloud);
    std::vector<pcl::PointIndices>::const_iterator it;
	std::vector<int>::const_iterator pit;
	//Declare array for storing the centers and radii of all obstacles.
	std::vector<Circle> objects;
	float xSum, ySum;
	for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		xSum = 0, ySum = 0;
        //PointCloud3D::Ptr cloud_cluster (new PointCloud3D);
        PointCloud3D::Ptr cluster (new PointCloud3D);
        for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        //push_back: add a point to the end of the existing vector
        		xSum += cloud3D_filtered->points[*pit].x;
        		ySum += cloud3D_filtered->points[*pit].y;
        		cluster->points.push_back(cloud3D_filtered->points[*pit]);
                //cloud_cluster->points.push_back(cloud3D_filtered->points[*pit]);
         		//std::cout << "(" << cloud_cluster->points[*pit].x << ", " << cloud_cluster->points[*pit].y << ")\n";
        }
        Circle obj;
        int size = it->indices.size();
        obj.centerX = xSum / size;
        obj.centerY = ySum / size;
        //Find the max distance from the center of the cluster
        float maxMagnitude = 0;
        for (int i = 0; i < cluster->points.size(); i++) {
        	float mag = sqrt(pow(cluster->points[i].x - obj.centerX, 2) + pow(cluster->points[i].y - obj.centerY, 2));
        	if (mag > maxMagnitude) {
        		maxMagnitude = mag;
        	}
        }
        obj.radius = maxMagnitude;
        objects.push_back(obj);
        clusters.push_back(cluster);
        //Merge current clusters to whole point cloud
    //*clustered_cloud += *cloud_cluster;

  }
  //Finish getting vectors here
  std::vector<Vector> distances;
  int numThreats = 0;
  for (int i = 0; i < objects.size(); i++) {
  		float distance = sqrt(pow(objects[i].centerX, 2) + pow(objects[i].centerY, 2)) - objects[i].radius;
  		//Robot is touching or inside the object
  		if (distance <= robotRadius) {
  			numThreats++;
  			//Compute the vector and store it in a object if it is a threat
  			float angle = atan(objects[i].centerY / objects[i].centerX);
  			float x = distance * cos(angle);
  			float y = distance * sin(angle);
  			Vector vec(x, y);
  			distances.push_back(vec);
  		}

  }
  for (int i = 0; i < walls.size(); i++) {
  	float distance = mag(walls[i].c) / sqrt(pow(walls[i].a, 2) + pow(walls[i].b, 2));
    float bottom = sqrt(pow(walls[i].a, 2) + pow(walls[i].b, 2));
    float top = mag(walls[i].c);
    std::cout << "Top: " << top << " Bottom: " << bottom << "\n";
  	//Mini algorithm to compute the vector between the origin and a line with given coefficients

  	if (distance <= robotRadius) {
  		float scale = -walls[i].c / (pow(walls[i].a, 2) + pow(walls[i].b, 2));
  		float slope = -walls[i].a / walls[i].b;
  		if (slope > 0) {
  			Vector vec(-mag(walls[i].a) * scale, mag(walls[i].b) * scale);
  			distances.push_back(vec);
  		} else {
  			Vector vec(mag(walls[i].a) * scale, mag(walls[i].b) * scale);
  			distances.push_back(vec);
  		}
  		numThreats++;
  	}
  }
	 if (numThreats == 0) {
  	std::cout << "No current obstacles found within range\n";
  }
  //Large negative number means close object on the left or bottom
  //Large positive number means close object on the right or top
  float vecXSum = 0, vecYSum = 0;
  for (int i = 0; i < distances.size(); i++) {
  		vecXSum += distances[i].x;
      std::cout << "vecXSum: " << vecXSum << "\n";
  		vecYSum += distances[i].y;
  }
  std::cout << "vecXSum value is: " << vecXSum << "\n" << "vecYSum value is: " << vecYSum << "\n";
  //Construct the message object to publish
  int isObstacle = 0;
  float motorLeft = 0;
  float motorRight = 0;
  if (numThreats > 0) {
  	isObstacle = 1;
  	if (vecXSum != 0) {
  		//Object is close on the right
  		if (vecXSum > 0) {
  			std::cout << "Turning left to avoid objects on the right\n";
  			motorRight = abs((1 / vecXSum) * xGain);
  		}
  		if (vecXSum < 0) {
  			std::cout << "Turning right to avoid objects on the left\n";
  			motorLeft = abs((1 / vecXSum) * xGain);
  		}
  		if (motorLeft == 0 && motorRight != 0) {
  			motorLeft = turnSharpness;
  		}
  		if (motorRight == 0 && motorLeft != 0) {
  			motorRight = turnSharpness;
  		}
  	}
    if (motorRight > 100) {
      motorRight = 100;
    }
    if (motorLeft > 100) {
      motorLeft = 100;
    }

    motorRight = motorRight*5 + 1500;
    motorLeft = motorLeft*5 + 1500;

  	//Problem here possibly
  	// if (vecYSum != 0) {
  	// 	if (vecYSum > 0) {
  	// 		std::cout << "Obstacle is really close to the front of the robot\n" << "Stopping...\n";
  	// 		motorLeft = 0;
  	// 		motorRight = 0;
  	// 	}
  	// 	if (vecYSum < 0) {
  	// 		std::cout << "Obstacle is really close to the back of the robot\n" << "Stopping...\n";
  	// 	}
  	// }
  	//Just need to publish the finished message
  }
	//Adjust the gain and output the right number here
	std::cout << "Publishing: \n" << "isObstacle: " << isObstacle << "\nmotorLeft: " << motorLeft << "\nmotorRight: " << motorRight << "\n";
  std_msgs::String msg;

  std::stringstream ss;

  ss << isObstacle << " " << motorLeft << " " << motorRight;

  msg.data = ss.str();

  std::cout << ss.str() << "\n";

//  std::string sisObstacle = boost::to_string(isObstacle);
//  std::string smotorLeft = boost::to_string(motorLeft);
//  std::string smotorRight = boost::to_string(motorRight);
//  msg.data = sisObstacle + " " + smotorLeft + " " + smotorRight;
  pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacleAvoidance");
    ros::NodeHandle n;

    pub = n.advertise<std_msgs::String>("avoid", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}
