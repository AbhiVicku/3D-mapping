#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>

using namespace std;

int main(){
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
	
	for(int i = 0;i <cloud.points.size();i++){
		cout << cloud.points[i]<< endl;
	}
	
	


	return 0;
}

