// example_node.cpp
// This is an example node to test the working of existing packages writen

#include <iostream>
#include <ros/ros.h>
#include "odomProcessor.h"
#include "pointCloudProcessor.h"
#include "poseGraph.h"

int main(int argc, char **argv)
{
	/* code */
	ros::init(argc,argv,"example_node");
	poseGraph::PoseGraph pg;
	
	return 0;
}
