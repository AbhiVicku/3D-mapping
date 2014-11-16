// example_node.cpp
// This is an example node to test the working of existing packages writen

#include <iostream>
#include <ros/ros.h>
#include "odomProcessor.h"
#include "pointCloudProcessor.h"
#include "poseGraph.h"
#include <Eigen/Dense>

int main(int argc, char **argv)
{
	/* code */
	int count = 0;
	Eigen::Matrix4f tr_mat;
	ros::init(argc,argv,"example_node");
	OdomProcess od_p;
	PointCloudProcessor pcl_p;
	PoseGraph pg;
	// Create initial subscriber 
	od_p.subr();
	pcl_p.subr();		

	/* while the process is going on do the stuff */
	while (ros::ok()){
		std::vector<double> odom_data;
		odom_data.push_back(od_p.pose_x);
		odom_data.push_back(od_p.pose_y);
		odom_data.push_back(od_p.yaw);		
		pg.addVertex(count,odom_data);
		if(pcl_p.cloud_seq_loaded.size()==2){
			
			pcl_p.calcICP();
			//pg.addEdgeToPrev(tr_mat);
			//std::cout << tr_mat << std::endl;
		}
		//pg.display();
		count++;
		ros::spinOnce();
	}
	
	
	return 0;
}
