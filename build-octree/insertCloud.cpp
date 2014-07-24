#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>


using namespace std;
using namespace octomap;

int main(){
	cout<<endl;
	cout<<" generating map "<<endl;
	
	// generating tree with resolution 0.1 m
	OcTree tree(0.1);
	
	// Read Point cloud from file 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& cloud = *cloud_ptr;
	if(pcl::io::loadPCDFile("test_pcd.pcd",cloud)==-1){
		cout<<"Error: file not found"<<endl;
		return(-1);
	}
	
	// filtering the initial point cloud 
	pcl::PassThrough<pcl::PointXYZ> pass(true);
	pass.setFilterFieldName("Z");
	pass.setFilterLimits(0.0,1.0);
	pass.filter(cloud);
	
	// printing the point cloud that was read 
	
	/*
	for(int i = 0;i <cloud.points.size();i++){
		cout << cloud.points[i].x<< endl;
	}*/
	
	
	//Insert the read point cloud to Octre structure
	octomap::Pointcloud pc; 
	octomap::point3d origin(0.0f,0.0f,0.0f);
	for(int i = 0;i<cloud.points.size();i++){
		pc.push_back((float) cloud.points[i].x,(float) cloud.points[i].y,(float) cloud.points[i].z);
	}
	
	tree.insertPointCloud(pc,origin, -1, false,false);
	
	
	
	/*
	// insert some measurements into it 
	for(int x = -20;x<20;x++){
		for(int y = -20;y<20;y++){
			for(int z = -20;z < 20;z++){
				point3d endpoint((float) x*0.02f -1.0f,(float) y*0.02f -1.0f,(float) z*0.02f - 1.0f);
				tree.updateNode(endpoint, true);
				
			}
		}
	}
	
	//insert some measurements of free cells into the previous tree
	for(int x = -30;x<30;x++){
		for(int y = -30;y<30;y++){
			for(int z = -30;z<30;z++){
				point3d endpoint ((float) x*0.02f-1.0f,(float) y*0.02f-1.0f,(float) z*0.02f - 1.0f);
				tree.updateNode(endpoint,false);
			}
		}
	
	}*/
	
	cout<<endl;
	tree.writeBinary("simple_tree.bt");
	cout<<"Finished"<<endl;
	return 0;
	

}
