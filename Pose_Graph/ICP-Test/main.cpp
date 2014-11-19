// This code demonstrates the ICP algo with pre-processing of point cloud for faster registration.

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/thread/thread.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/don.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

using namespace std;


Eigen::Matrix4f tr_mat; 


void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_0_ptr = cloud_in;
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
	
  	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_0_ptr));
  

  	std::vector<int> inliers;
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());	
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_0_ptr, inliers, *final);
  	
  	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_hull (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(final);     
	chull.setDimension(2);
    chull.reconstruct(*ground_hull);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    pcl::PointIndices::Ptr cloud_indices (new pcl::PointIndices);
	prism.setInputCloud (cloud_0_ptr);
	prism.setInputPlanarHull (ground_hull);
	prism.setHeightLimits (0.05, 5.0);
	prism.segment (*cloud_indices);

	pcl::ExtractIndices<pcl::PointXYZ> eifilter (true);
	eifilter.setInputCloud (cloud_0_ptr);
	eifilter.setIndices (cloud_indices);
	eifilter.filterDirectly (cloud_0_ptr);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
	dy_sor.setInputCloud (cloud_0_ptr);
	dy_sor.setMeanK (20);
	dy_sor.setStddevMulThresh (0.5);
	dy_sor.filter (*cloud_0_ptr);

}



int main(int argc,char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& cloud_1 = *cloud_1_ptr;
	std::vector<int> nan_indices;
	
	
	
	
	
	
	if(pcl::io::loadPCDFile("config_0.pcd",cloud_1)==-1){
		cout<<"Error: file not found"<<endl;
		return(-1);
	}
	
	pcl::PassThrough<pcl::PointXYZ> pass_1;
	pass_1.setInputCloud (cloud_1_ptr);
	pass_1.setFilterFieldName ("z");
	pass_1.setFilterLimits (0.0, 3.0);
	pass_1.filter (*cloud_1_ptr);

	filterCloud(cloud_1_ptr);
	pcl::removeNaNFromPointCloud(cloud_1,cloud_1,nan_indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_15_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& cloud_2 = *cloud_15_ptr;
	
	if(pcl::io::loadPCDFile("config_30.pcd",cloud_2)==-1){
		cout<<"Error: file not found"<<endl;
		return(-1);
	}
	std::vector<int> nan_indices1;
	
	pcl::PassThrough<pcl::PointXYZ> pass_2;
	pass_2.setInputCloud (cloud_15_ptr);
	pass_2.setFilterFieldName ("z");
	pass_2.setFilterLimits (0.0, 3.0);
	pass_2.filter (*cloud_15_ptr);

	filterCloud(cloud_15_ptr);
	pcl::removeNaNFromPointCloud(cloud_2,cloud_2,nan_indices1);
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_1_ptr);
	icp.setInputTarget(cloud_15_ptr);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	
	/*
	pcl::VoxelGrid<pcl::PointXYZ> vxl_;
	vxl_.setInputCloud (cloud_0_ptr);
	vxl_.setLeafSize(0.005,0.005,0.005);
	vxl_.filter (*cloud_0_ptr);	
	*/
	
  	pcl::visualization::CloudViewer viewer1 ("Simple Cloud Viewer1");
  	viewer1.showCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(Final));
  	viewer1.showCloud(cloud_1_ptr);
  	while (!viewer1.wasStopped ())
  	{
    	}
  	
	return 0;
}

