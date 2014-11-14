/*
 * pointCloudProcessor.h
 * Description : Header file for pointCloudProcessor.cpp
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
#ifndef __POINTCLOUDPROCESSOR_H_INCLUDED__  // if this header is not included yet
#define __POINTCLOUDPROCESSOR_H_INCLUDED__  // then include this header

#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

class PointCloudProcessor{
	private:
		//different types of clouds are initialized 
		pcl::PCLPointCloud2::Ptr cloud_ptr_ ;
		pcl::PCLPointCloud2 cloud_ ; 
		// initializing only pointer without memory allocations results in error
		
		//*cloud_ptr_ = cloud_;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc_ptr_ ;
		pcl::PointCloud<pcl::PointXYZ> curr_pc_;
		//*curr_pc_ptr_ = curr_pc_;
		
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
		Eigen::Matrix4f tr_mat_;
	public:
		//constructor
		PointCloudProcessor();
		void pclCallbk(sensor_msgs::PointCloud2 msg);
		//void fetchCloud();
		void filterCloud();
		//Calculate Iterative closest point matching between the 2 point clouds
		Eigen::Matrix4f calcICP(pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc,pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc);		
	};
	
#endif	
