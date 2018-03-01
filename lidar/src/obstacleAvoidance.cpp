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
typedef pcl::PointXY  Point2D;
typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud<Point2D> PointCloud2D;
typedef pcl::PointCloud<Point3D> PointCloud3D;
//The range at which all objects should keep away from
float robotRadius;
ros::Publisher pub;
int frameCount = 0;
//The numbers to filter out bad values
float range_min = 0;
float range_max = 10000;
float xGain, yGain;
class Vector {
public:
	float x, y;
	Vector (float xx, yy) {
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
    std::cout << "Filtered cloud using sor size: " << cloud3D_filtered->points.size() << "\n";
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
    	std::cout << "Coefficients: " << "\n";
    	seg.setInputCloud (cloud3D_filtered);
    	seg.segment (*inliers, *coefficients);
    	for (int i = 0; i < coefficients->values.size(); i++) {
    		std::cout << (i + 1) << ": " << coefficients->values[i] << "\n";
    	}
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
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points\n";
    	//Remove the planar inliers, extract the rest.
    	 extract.setNegative (true);
    	 extract.filter (*cloud_f);
    	 *cloud3D_filtered = *cloud_f;
     }
    //std::cout << "The number of coefficients is: " << coefficients_group.size() << "\n";
    std::cout << "Cloud_filtered size:" << cloud3D_filtered->points.size() << "\n";
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
        std::cout << "New cluster\n";
        std::cout << "--------------------------\n";
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
  		}

  }
  for (int i = 0; i < walls.size(); i++) {
  	float distance = abs(walls[i].c) / sqrt(pow(walls[i].a, 2) + pow(walls[i].b, 2));
  	if (distance <= robotRadius) {
  		numThreats++;
  	}
  }
  //Large negative number means close object on the left or bottom
  //Large positive number means close object on the right or top
  float vecXSum = 0, vecYSum = 0;
  for (int i = 0; i < distances.size(); i++) {
  		vecXSum += distances[i].x;
  		vecYSum += distances[i].y;
  }
	//Adjust the gain and output the right number here   
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
