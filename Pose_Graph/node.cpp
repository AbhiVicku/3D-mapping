/*
 * node.cpp
 * 
 * Copyright 2014 abhinav <abhinav@abhinav-VirtualBox>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


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
#include <pcl/registration/icp.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <Eigen/Dense>

typedef std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq;
typedef std::deque<std::list<double> > pose_seq;
//create custom vertex property
struct Pose{
	int key;
	std::list<double>  data;
	};
struct constraints{
	Eigen::Matrix4f transformation;
	
	};

//Property types 
typedef boost::property<boost::edge_weight_t,int> EdgeWeightProperty;
typedef boost::property<boost::vertex_name_t,std::string,boost::property<boost::vertex_index2_t,int> > VertexProperties;

//Graph type 
//typedef boost::adjacency_list<boost::vecS,boost::vecS, boost::undirectedS,VertexProperties, EdgeWeightProperty> Graph;
typedef boost::adjacency_list<boost::listS,boost::vecS, boost::undirectedS,Pose, constraints> Graph;

// create hte vertices
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

//create edges 
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

Graph g;
Eigen::Matrix4f tr_mat;
double roll, pitch, yaw;
double pose_x,pose_y;
cloud_seq cloud_seq_loaded; 
pose_seq pose_seq_loaded;
const float res = 0.1;
pcl::PointCloud<pcl::PointXYZ> static_pc;

void createGraph(const Eigen::Matrix4f tr_msg){
	
	
	Vertex v1 ;
	v1 = boost::add_vertex(g);
	g[v1].key = 40;
	g[v1].data  = pose_seq_loaded[0];
	
	Vertex v2 ;
	v2 = boost::add_vertex(g);
	g[v2].key = 21;
	g[v2].data = pose_seq_loaded[1];
	
	
	Edge e1; 
	e1 = (boost::add_edge(v1,v2,g)).first;
	g[e1].transformation = tr_msg;
	
	
	
	//Iterate over the graph 
	Graph::vertex_iterator vertexIt,vertexEnd;
	Graph::adjacency_iterator neighbourIt, neighbourEnd;
	
	
	boost::tie(vertexIt, vertexEnd) = boost::vertices(g);
	for(;vertexIt!=vertexEnd;++vertexIt)
	{
		std::cout<<g[*vertexIt].key<<"is connected with";
		boost::tie(neighbourIt,neighbourEnd) = boost::adjacent_vertices(*vertexIt,g);
		for(;neighbourIt != neighbourEnd;++neighbourIt)
		std::cout<<g[*neighbourIt].key<<" ";
		std::cout<<std::endl;
		}
	Graph::edge_iterator edgeIt,edgeEnd;
	boost:tie(edgeIt,edgeEnd) = boost::edges(g);
	for(;edgeIt!=edgeEnd;++edgeIt){
		std::cout<<g[*edgeIt].transformation<<std::endl;
		
		}
	
	
	}

void getOdom(nav_msgs::Odometry data){
	/*
	 *This function gets the odometry msg from the robot 
	 * and using TF gives the angle in euclidean form. 
	 * 
	 */
	
	//std::cout<<"Receiving odometry"<<std::endl;
	//std::cout<< data.pose.pose.orientation<<std::endl;
	tf::Quaternion q(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	pose_x = data.pose.pose.position.x;
	pose_y = data.pose.pose.position.y;
	m.getRPY(roll, pitch, yaw);

	//std::cout << "x: " << pose_x << ", y : " << pose_y << ", Yaw: " << yaw << std::endl;
	//std::cout<< v[1];	
	}
		
void processCloud(const sensor_msgs::PointCloud2 msg)
{
	std::cout<<"Receiving cloud"<<std::endl;
	std::cout << "x: " << pose_x << ", y : " << pose_y << ", Yaw: " << yaw << std::endl;
	double myPoses[] = {pose_x,pose_y,yaw};
	std::list<double> curr_pose (myPoses, myPoses + sizeof(myPoses) / sizeof(double) );
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud;
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	std::vector<int> nan_indices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pc (new pcl::PointCloud<pcl::PointXYZ>); 
		
	
	//********* Retirive and process raw pointcloud************
	pcl_conversions::toPCL(msg,cloud);
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	*curr_pc =pcl_pc;
	
	//***************Filter point cloud to detect nearby changes only *****************
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (curr_pc);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 3.0);
		pass.filter (*curr_pc);
		
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
		dy_sor.setInputCloud (curr_pc);
		dy_sor.setMeanK (50);
		dy_sor.setStddevMulThresh (1.0);
		dy_sor.filter (*curr_pc);
	
	cloud_seq_loaded.push_back(pcl_pc);
	pose_seq_loaded.push_back(curr_pose);
	std::cout<<pose_seq_loaded.size()<<std::endl;
	
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
		pose_seq_loaded.pop_front();
	
	}
	if(cloud_seq_loaded.size()==1){
		static_pc = pcl_pc;
		}
	if(cloud_seq_loaded.size()==2)
	{
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
		*first_pc = cloud_seq_loaded[0];
		*second_pc = cloud_seq_loaded[1];
		icp.setInputSource(second_pc);
		icp.setInputTarget(first_pc);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//icp.getFitnessScore() << std::endl;
		//std::cout << icp.getFinalTransformation() << std::endl;
		tr_mat = icp.getFinalTransformation();
		std::cout<<tr_mat<<std::endl;
		createGraph(tr_mat);
		}
		
	/*
	//*********** Remove old data, update the data***************	
	cloud_seq_loaded.push_back(pcl_pc);
	std::cout<<cloud_seq_loaded.size()<<std::endl;
	
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
	
	}
	if(cloud_seq_loaded.size()==1){
		static_pc = pcl_pc;
		}
	
	//*********** Process currently observerd and buffered data*********

	if(cloud_seq_loaded.size()==2){
		
		
		*prev_pc =static_pc; //cloud_seq_loaded.front(); ss
		
		//*************Create octree structure and search
		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (0.5);
		octree.setInputCloud(prev_pc);
		octree.addPointsFromInputCloud();
		octree.switchBuffers();
		octree.setInputCloud(curr_pc);
		octree.addPointsFromInputCloud();
		std::vector<int> newPointIdxVector;
		// Get vector of point indices from octree voxels which did not exist in previous buffer
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);
		std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_points (new pcl::PointCloud<pcl::PointXYZ>);
		dynamic_points->header.frame_id = "some_tf_frame";
		
		for (size_t i = 0; i < newPointIdxVector.size (); ++i){
			pcl::PointXYZ point;
			point.x = pcl_pc.points[newPointIdxVector[i]].x;
			point.y = pcl_pc.points[newPointIdxVector[i]].y;
			point.z = pcl_pc.points[newPointIdxVector[i]].z;
			dynamic_points->push_back(point);
			//std::cout << i << "# Index:" << newPointIdxVector[i]<< "  Point:" << pcl_pc.points[newPointIdxVector[i]].x << " "<< pcl_pc.points[newPointIdxVector[i]].y << " "<< pcl_pc.points[newPointIdxVector[i]].z << std::endl;
			
		}	
		std::cout<<newPointIdxVector.size ()<<std::endl;
		
		//***************Filter point cloud to detect nearby changes only *****************
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (dynamic_points);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 3.0);
		pass.filter (*dynamic_points);
		
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
		dy_sor.setInputCloud (dynamic_points);
		dy_sor.setMeanK (50);
		dy_sor.setStddevMulThresh (1.0);
		dy_sor.filter (*dynamic_points);
		
		//**********************Publish the data************************************
		ros::NodeHandle k;
		ros::Publisher pub = k.advertise<pcl::PointCloud<pcl::PointXYZ> >("dynamicPoints",2);
		pub.publish(dynamic_points);
		ros::Time time = ros::Time::now();
		//Wait a duration of one second.
		ros::Duration d = ros::Duration(1.5, 0);
		d.sleep();
		ros::spinOnce();
		
				

		}
	*/ 
	
	std::cout<<"finished"<<std::endl;
	std::cout<<std::endl;	
	}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "pose_graph");
	std::cout<<"Create object of posegraph"<<std::endl;
	ros::NodeHandle nh;//create handle
	uint32_t queue_size = 1;
	ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/odom",queue_size,getOdom);
	ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	ros::spin();	
	return 0;
}

