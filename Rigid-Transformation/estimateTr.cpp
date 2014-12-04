/*
 * estimateTr.cpp
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
#include "sensor_msgs/PointCloud2.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <deque>
#include <pcl/octree/octree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded; 
const float res = 0.1;



void processCloud(const sensor_msgs::PointCloud2 msg)
{
	//********* Retirive and process raw pointcloud************
	std::cout<<"Recieved cloud"<<std::endl;
	//std::cout<<"Create Octomap"<<std::endl;
	//octomap::OcTree tree(res);
	//std::cout<<"Load points "<<std::endl;
	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(msg,cloud);
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	//std::cout<<"Filter point clouds for NAN"<<std::endl;
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	//octomap::Pointcloud oct_pc;
	//octomap::point3d origin(0.0f,0.0f,0.0f);
	//std::cout<<"Adding point cloud to octomap"<<std::endl;
	//octomap::point3d origin(0.0f,0.0f,0.0f);
	//for(int i = 0;i<pcl_pc.points.size();i++){
		//oct_pc.push_back((float) pcl_pc.points[i].x,(float) pcl_pc.points[i].y,(float) pcl_pc.points[i].z);
	//}
	//tree.insertPointCloud(oct_pc,origin,-1,false,false);
	
	//*********** Remove the oldest data, update the data***************	
	cloud_seq_loaded.push_back(pcl_pc);
	std::cout<<cloud_seq_loaded.size()<<std::endl;
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
	
	}
	
	
	//*********** Process currently observerd and buffered data*********

	if(cloud_seq_loaded.size()==2){
		//std::cout<< "Generating octomap"<<std::endl;
		std::cout<<"Ransac"<<std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pc (new pcl::PointCloud<pcl::PointXYZ>); 
		*prev_pc = cloud_seq_loaded.front();
		pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZ>);
		*curr_pc =pcl_pc;
		
		pcl::RandomSample<pcl::PointXYZ> rs(true);
		rs.setSample(20000);
		rs.setInputCloud(curr_pc);
		std::vector<int> indices;
		rs.filter (indices);
		pcl::PointCloud<pcl::PointXYZ> curr_cld_out;
		rs.filter(curr_cld_out);
		*curr_pc = curr_cld_out;
		std::cout<<curr_pc->size()<<std::endl;
		rs.setInputCloud(prev_pc);
		rs.filter(indices);
		pcl::PointCloud<pcl::PointXYZ> prev_cld_out;
		rs.filter(prev_cld_out);
		*prev_pc = prev_cld_out;
		/*
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> esTrSVD;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 trMat;
		esTrSVD.estimateRigidTransformation(*prev_pc,*curr_pc,trMat);
		std::cout<<trMat<<std::endl;
		
		*/
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		//pcl::registration::TransformationEstimationPointToPlaneLLS < pcl::PointXYZ,pcl::PointXYZ,float>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlaneLLS < pcl::PointXYZ,pcl::PointXYZ,float>);
		pcl::registration::TransformationEstimationLM< pcl::PointXYZ,pcl::PointXYZ,float >::Ptr trans_LM (new pcl::registration::TransformationEstimationLM< pcl::PointXYZ,pcl::PointXYZ,float >); 
		icp.setInputSource(prev_pc);
		icp.setInputTarget(curr_pc);
		icp.setTransformationEstimation (trans_LM);
		//icp.setMaximumIterations (2);
		//icp.setRANSACIterations(5);
		icp.setTransformationEpsilon (1e-8);
		icp.setMaxCorrespondenceDistance (0.5);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 trMat;
		trMat = icp.getFinalTransformation();
		
		//*curr_pc = pcl_pc;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::transformPointCloud (*prev_pc, *transformed_cloud, trMat);
		pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");	
		 // Define R,G,B colors for the point cloud
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (prev_pc, 255, 255, 255);
		// We add the point cloud to the viewer and pass the color handler
		viewer.addPointCloud (prev_pc, source_cloud_color_handler, "original_cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
		viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
		
		viewer.addCoordinateSystem (1.0, 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
		//viewer.setPosition(800, 400); // Setting visualiser window position

		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce ();
		}
			//std::cout << "Node center: " << it.getCoordinate() << std::endl;
			//std::cout << "Node size: " << it.getSize() << std::endl;
			//std::cout << "Node value: " << it->getValue() << std::endl;
				

		}
		//********** print out the statistics ******************
		
	
	//**************Process Point cloud in octree data structure *****************
	
	
	
	
	
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
	std::cout<<"finished"<<std::endl;
	std::cout<<std::endl;
	}


int main(int argc, char **argv)
{
	std::cout<<std::endl;
	std::cout<<"initializing 2D rigid transformation node"<<std::endl;
	ros::init(argc,argv, "processCloud");
	ros::NodeHandle n;
	std::cout<<"subscribing to topic point_cloud"<<std::endl;
	//std::string topic =n.resolveName("point_cloud");
	uint32_t queue_size = 1;
	
	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	ros::spin(); 
	return 0;
}

