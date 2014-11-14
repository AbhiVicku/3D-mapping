/*
 * pointCloudProcessor.cpp
 * 
 * Copyright 2014 Shibata-Lab <shibata-lab@shibatalab-X500H>
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

#include "pointCloudProcessor.h"
// constructor	
PointCloudProcessor::PointCloudProcessor(){
	
	uint32_t queue_size = 1;
	sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,&PointCloudProcessor::pclCallbk, this);
	
	}	

// Callback function for sensor_msg pointcloud	
void PointCloudProcessor::pclCallbk(sensor_msgs::PointCloud2 msg){
	std::vector<int> nan_indices;
	pcl_conversions::toPCL(msg,cloud_);
	pcl::fromPCLPointCloud2(cloud_,curr_pc_);
	pcl::removeNaNFromPointCloud(curr_pc_,curr_pc_,nan_indices);
	}

// Filter processor 	
void PointCloudProcessor::filterCloud(){
	// parameter values are still yet to be tested for best results
	*curr_pc_ptr_ = curr_pc_;
	pcl::PassThrough<pcl::PointXYZ> pass_;
	pass_.setInputCloud (curr_pc_ptr_);
	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (0.0, 3.0);
	pass_.filter (*curr_pc_ptr_);
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
	sor_.setInputCloud (curr_pc_ptr_);
	sor_.setMeanK (20);
	sor_.setStddevMulThresh (1.0);
	sor_.filter (*curr_pc_ptr_);
	return;
	}
Eigen::Matrix4f calcICP(pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc,pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc){
	Eigen::Matrix4f tr_mat;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(first_pc);
	icp.setInputTarget(second_pc);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	tr_mat = icp.getFinalTransformation();
	return tr_mat;
	}

/*			
int main(int argc, char **argv)
{	ros::init(argc, argv, "PointCloudrocessor");
	PointCloudProcessor cld_process();
	
	return 0;
}*/

