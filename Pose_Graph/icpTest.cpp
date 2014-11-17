

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <deque>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <Eigen/Dense>
#include <boost/graph/graphviz.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>


std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded;
Eigen::Matrix4f tr_mat; 

void algoICP(pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc){
	std::cout << "2"<<std::endl;	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(first_pc);
	icp.setInputTarget(second_pc);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	tr_mat = icp.getFinalTransformation();
		
	//std::cout<<tr_mat<<std::endl;
			
}

/*
void filterCloud(pcl::pointcloud<pcl::PointXYZ>::Ptr  in_cloud){
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

}
*/		
void processCloud(const sensor_msgs::PointCloud2 msg)
{
	std::cout<<"Receiving cloud"<<std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud;
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	std::vector<int> nan_indices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pc (new pcl::PointCloud<pcl::PointXYZ>); 
		
	
	//********* Retirive and process raw pointcloud************
	pcl_conversions::toPCL(msg,cloud);
	pcl::PCLPointCloud2::Ptr cloud_ptr  (new pcl::PCLPointCloud2 ());
	*cloud_ptr = cloud;
	/*
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_ptr);
	sor.setLeafSize (0.05f, 0.05f, 0.05f);
	sor.filter (*cloud_ptr);
	*/
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	*curr_pc =pcl_pc;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
	dy_sor.setInputCloud (curr_pc);
	dy_sor.setMeanK (20);
	dy_sor.setStddevMulThresh (1.0);
	dy_sor.filter (*curr_pc);
	
	cloud_seq_loaded.push_back(pcl_pc);

	
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
	}
	if(cloud_seq_loaded.size()==2)
	{
		std::cout << "1"<<std::endl;	
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
		*first_pc = cloud_seq_loaded[0];
		*second_pc = cloud_seq_loaded[1];
		algoICP(first_pc,second_pc);	
		}
		
	 
	
	std::cout<<"finished"<<std::endl;
	std::cout<<std::endl;	
	}




int main(int argc, char **argv)
{	
	
	// Initialize ros nodes
	ros::init(argc, argv, "pose_graph");
	ros::NodeHandle nh;//create handle
	uint32_t queue_size = 1;
	
	/*
	 * Now, Point cloud is subscribed to from the kinect sensor, however this acquisition is slow and needs to be updated
	 */ 
	ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	
	ros::spin();	
	return 0;
}

