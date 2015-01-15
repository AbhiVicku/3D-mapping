#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <deque>
#include <string>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

const float res = 0.05; // resolution for the octomap
int count =1; 	// counter for the file names




// Ros msg callback for sensor_msgs
void processCloud(const sensor_msgs::PointCloud2 msg)
{
	//********* Retirive and process raw pointcloud************
	// Initialize octomap with given resolution. 
	octomap::OcTree tree(res);
	octomap::Pointcloud oct_pc;
	octomap::point3d origin(0.0f,0.0f,0.0f);
	
	// Retrieve cloud msg and convert it to usable cloud for ROS.
	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(msg,cloud);
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_0_ptr = pcl_pc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// filter point cloud 
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
	dy_sor.setInputCloud (cloud_0_ptr);
	dy_sor.setMeanK (20);
	dy_sor.setStddevMulThresh (0.5);
	dy_sor.filter (*cloud_0_ptr);

	pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setInputCloud (cloud_0_ptr);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (0.0, 2.0);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*cloud_filtered);
	// Add the given point cloud to octomap Pointcloud data structure. 
	for(int i = 0;i<cloud_filtered->points.size();i++){
		oct_pc.push_back((float) cloud_filtered->points[i].x,(float) cloud_filtered->points[i].y,(float) cloud_filtered->points[i].z);
	}
	
	// Insert complete octomap point cloud to the Octree structure.
	tree.insertPointCloud(oct_pc,origin,-1,false,false);
	
	/*
	//******************Traverse the tree ********************
	for(octomap::OcTree::tree_iterator it =tree.begin_tree(), end = tree.end_tree();it!= end;it++){
		 //manipulate node, e.g.:
		std::cout << "_____________________________________"<<std::endl;
		std::cout << "Node center: " << it.getCoordinate() << std::endl;
		std::cout << "Node size: " << it.getSize() << std::endl;
		std::cout << "Node depth: "<<it.getDepth() << std::endl;
		std::cout << "Is Leaf : "<< it.isLeaf()<< std::endl;
		std::cout << "_____________________________________"<<std::endl;
		}
	//**********************************************************	
	*/

	// saving the point cloud
	std::cout<<"Save the pointcloud"<<std::endl;
	int ans;
	std::cin>>ans;
	std::ostringstream sstream;
	std::string str ="savedCloud";
	if(ans==0){
		sstream<<str<<count<<".bt";
		tree.writeBinary(sstream.str());	
		count++;
	}	
		
	}


int main(int argc, char **argv){	

	// Initializing ros parameters and subscribing to the topic.
	ros::init(argc,argv, "processCloud");
	ros::NodeHandle n;
	std::cout<<"subscribing to topic point_cloud"<<std::endl;
	uint32_t queue_size = 1;	
	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	ros::spin(); 
	return 0;
}
