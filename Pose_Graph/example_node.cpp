// example_node.cpp
// This is an example node to test the working of existing packages writen. It is still in development phase

#include <iostream>
#include <ros/ros.h>
#include "odomProcessor.h"
#include "pointCloudProcessor.h"
#include "poseGraph.h"
#include <Eigen/Dense>

int main(int argc, char **argv)
{
	/* code */
	int count ;
	bool first;
	Eigen::Matrix4f tr_mat;
	ros::init(argc,argv,"example_node");
	OdomProcess od_p;
	PointCloudProcessor pcl_p;
	PoseGraph pg;
	// Create initial subscriber 
	od_p.subr();
	pcl_p.subr();		
	first = true;
	count = 0;
	
	/* while the process is going on do the stuff */
	while (ros::ok()){
		std::vector<double> odom_data;
		odom_data.push_back(od_p.pose_x);
		odom_data.push_back(od_p.pose_y);
		odom_data.push_back(od_p.yaw);		
		
		if(pcl_p.cloud_seq_loaded.size()==1){
			std::cout<<"Initalizing pose graph"<< std::endl;
			std::vector<double> odom_init;
			odom_init.push_back(od_p.pose_x);
			odom_init.push_back(od_p.pose_y);
			odom_init.push_back(od_p.yaw);	

			pg.addVertex(count, odom_init);
		}

		if(pcl_p.cloud_seq_loaded.size()==2){
				std::cout<<"count is :"<< count<< std::endl;
				
				tr_mat = pcl_p.calcICP();
				//std::cout << pcl_p.calcICP() <<std::endl;
				count++;
				pg.addVertex(count,odom_data);
				pg.addEdgeToPrev(tr_mat);
				//std::cout << tr_mat << std::endl;
				
		}
		

		pg.display();
		
		ros::spinOnce();
	}
	
	
	return 0;
}
