/*
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
 */



#include "octomap_sub.h"

std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded; 
const float res = 0.1;
pcl::PointCloud<pcl::PointXYZ> static_pc;





void processCloud(const sensor_msgs::PointCloud2 msg)
{
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
	std::cout<<"finished"<<std::endl;
	std::cout<<std::endl;
	}


int main(int argc, char **argv){	
	std::cout<<std::endl;
	ros::init(argc,argv, "processCloud");
	ros::NodeHandle n;
	uint32_t queue_size = 1;
	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	ros::spin(); 
	return 0;
}
