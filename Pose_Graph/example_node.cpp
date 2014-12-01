// example_node.cpp
// This is an example node to test the working of existing packages writen. It is still in development phase

#include <iostream>
#include <ros/ros.h>
#include "odomProcessor.h"
#include "pointCloudProcessor.h"
#include "poseGraph.h"
#include <Eigen/Dense>
#include <fstream>


int main(int argc, char **argv)
{
	std::cout << "----------------------------------------------- "<<std::endl;
	std::cout << "-------------This is a pose graph slam--------- "<<std::endl;
	std::cout << "----------------------------------------------- "<<std::endl;
	
	std::ofstream myfile;
	/* code */
	myfile.open ("example.txt");
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
		
		if((pcl_p.cloud_seq_loaded.size()==1) && first){
			std::cout<<"-->Initalizing pose graph"<< std::endl;
			std::vector<double> odom_init;
			odom_init.push_back(od_p.pose_x);
			odom_init.push_back(od_p.pose_y);
			odom_init.push_back(od_p.yaw);	

			pg.addVertex(count, odom_init);
			first = false;
		}

		if(pcl_p.cloud_seq_loaded.size()==2){
				std::cout<<"Count is :"<< count<< std::endl;
				
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
	
	
	boost::tie(pg.vertexIt_, pg.vertexEnd_) = boost::vertices(pg.gr_);
	for(;pg.vertexIt_!=pg.vertexEnd_;++pg.vertexIt_)
	{
		myfile<<"Vertex ";
		myfile<<pg.gr_[*pg.vertexIt_].key;
		myfile<<" ";
		for (int i =0;i< 3;i++){
			myfile<<pg.gr_[*pg.vertexIt_].data[i];
			std::cout<<pg.gr_[*pg.vertexIt_].data[i]<<std::endl;;
			myfile<<" ";
			}
		myfile<<"\n";
		
		std::cout<<std::endl;
		}
	boost::tie(pg.edgeIt_,pg.edgeEnd_) = boost::edges(pg.gr_);
	for(;pg.edgeIt_!=pg.edgeEnd_;++pg.edgeIt_)
	{
		//myfile<<*pg.vertexIt_<;
		myfile<<"Edge ";
		Eigen::Matrix4f trfMat = pg.gr_[*pg.edgeIt_].transformation;
		for(int i = 0;i<16;i++){
			myfile<< *(trfMat.data()+i)<< " ";
		} 
		/* 
		for(size_t i =0;i<=trfMat.rows();i++ ){
			for (size_t j = 0; j<=trfMat.cols();j++){
				myfile<< *(trfMat.data()+i+j)<< " ";;	
			}
			
			}*/
		//myfile<<pg.gr_[*pg.edgeIt_].transformation;
		myfile<< "\n";
		
		}
	
	
	/*
	boost::tie(neighbourIt,neighbourEnd) = boost::adjacent_vertices(*vertexIt,g);
		for(;neighbourIt != neighbourEnd;++neighbourIt)
		std::cout<<*neighbourIt<<" ";
	*/
	
	myfile.close();
	
	return 0;
}
