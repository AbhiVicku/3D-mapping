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
//TODO : 

#include "pointCloudProcessor.h"
// constructor	
void PointCloudProcessor::subr(){
	std::cout<< "2" << std::endl;
	uint32_t queue_size = 1;
	sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,&PointCloudProcessor::pclCallbk, this);
	
	}	

// Callback function for sensor_msg pointcloud	
void PointCloudProcessor::pclCallbk(sensor_msgs::PointCloud2 msg){
	//std::cout<< "subscribing to pcl" <<std::endl;
	std::cout<< "4" <<std::endl;
	std::vector<int> nan_indices;
	pcl_conversions::toPCL(msg,cloud_);
	pcl::fromPCLPointCloud2(cloud_,curr_pc_);
	pcl::removeNaNFromPointCloud(curr_pc_,curr_pc_,nan_indices);
	curr_pc_ = this->filterCloud();
	cloud_seq_loaded.push_back(curr_pc_);
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
		//pose_seq_loaded.pop_front();
	}
	/*
	for (size_t i = 0; i < curr_pc_.size();i++){
		std::cout << curr_pc_.points[i]<<std::endl;
	}*/
}

// Filter processor 	
pcl::PointCloud<pcl::PointXYZ> PointCloudProcessor::filterCloud(){
	// parameter values are still yet to be tested for best results
	pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc_ptr_ (new pcl::PointCloud<pcl::PointXYZ>);
	*curr_pc_ptr_= curr_pc_;
	pcl::VoxelGrid<pcl::PointXYZ> vxl_;
	vxl_.setInputCloud (curr_pc_ptr_);
	vxl_.setLeafSize(0.05,0.05,0.05);
	vxl_.filter (*curr_pc_ptr_);

	/*
	pcl::PassThrough<pcl::PointXYZ> pass_;
	pass_.setInputCloud (curr_pc_ptr_);
	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (0.0, 3.0);
	pass_.filter (*curr_pc_ptr_);
	*/
	pcl::ModelCoefficients ground_coefficients;
	pcl::PointIndices ground_indices;
	pcl::SACSegmentation<pcl::PointXYZ> ground_finder;
  	ground_finder.setOptimizeCoefficients(true);
  	ground_finder.setModelType(pcl::SACMODEL_PLANE);
  	ground_finder.setMethodType(pcl::SAC_RANSAC);
  	ground_finder.setDistanceThreshold(0.015);
  	ground_finder.setInputCloud(curr_pc_ptr_);
	ground_finder.segment(ground_indices, ground_coefficients);


  	// Step 3a. Extract the ground plane inliers
  	pcl::PointCloud<pcl::PointXYZ> ground_points;
  	pcl::ExtractIndices<pcl::PointXYZ> extractor;
  	extractor.setInputCloud(curr_pc_ptr_);
  	extractor.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
  	extractor.filter(ground_points);

  	// Step 3b. Extract the ground plane outliers
 	pcl::PointCloud<pcl::PointXYZ> object_points;
  	pcl::ExtractIndices<pcl::PointXYZ> outlier_extractor;
  	outlier_extractor.setInputCloud(curr_pc_ptr_);
  	outlier_extractor.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
  	outlier_extractor.setNegative(true);
  	outlier_extractor.filter(object_points);


	 // Step 3c. Project the ground inliers

	pcl::PointCloud<pcl::PointXYZ> cloud_projected;
  	pcl::ProjectInliers<pcl::PointXYZ> proj;
 	proj.setModelType(pcl::SACMODEL_PLANE);
  	proj.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(ground_points));
  	proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(ground_coefficients));
  	proj.filter(cloud_projected);

  	// Step 3d. Create a Convex Hull representation of the projected inliers
  	pcl::PointCloud<pcl::PointXYZ> ground_hull;
  	pcl::ConvexHull<pcl::PointXYZ> chull;
  	chull.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_projected));
  	chull.reconstruct(ground_hull);

  	ROS_INFO ("Convex hull has: %d data points.", (int) ground_hull.points.size ());

  
  	// Step 3e. Extract only those outliers that lie above the ground plane's convex hull
  	pcl::PointIndices object_indices;
  	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> hull_limiter;
  	hull_limiter.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(object_points));
  	hull_limiter.setInputPlanarHull(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(ground_hull));
  	hull_limiter.setHeightLimits(0, 0.6);
  	hull_limiter.segment(object_indices);

  	pcl::ExtractIndices<pcl::PointXYZ> object_extractor;
  	object_extractor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(object_points));
  	object_extractor.setIndices(boost::make_shared<pcl::PointIndices>(object_indices));
  	object_extractor.filter(object_points);
  	pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("ExtractedPoints",2);
  	pub_.publish(object_points);
  	return object_points;
	/*
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
	sor_.setInputCloud (curr_pc_ptr_);
	sor_.setMeanK (20);
	sor_.setStddevMulThresh (1.0);
	sor_.filter (*curr_pc_ptr_);
	*/
	
	}
/* USing gicp */
/*
Eigen::Matrix4f PointCloudProcessor::calcICP(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
	*first_pc = cloud_seq_loaded[0];
	*second_pc = cloud_seq_loaded[1];
	Eigen::Matrix4f tr_mat;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	gicp.setInputSource(first_pc);
	gicp.setInputTarget(second_pc);
	pcl::PointCloud<pcl::PointXYZ> Final;
	gicp.align(Final);
	tr_mat = gicp.getFinalTransformation();
	return tr_mat;
	}
*/

/*
// USing ICP from PCL 

Eigen::Matrix4f PointCloudProcessor::calcICP(){
	bool flag = true;

	pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
	*first_pc = cloud_seq_loaded[0];
	*second_pc = cloud_seq_loaded[1];
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::Registration<pcl::PointXYZ,pcl::PointXYZ,float tmp>::TransformationEstimationPointToPlaneLLS < pcl::PointXYZ,pcl::PointXYZ>::Ptr trans_lls (new pcl::Registration<pcl::PointXYZ,pcl::PointXYZ,float tmp>::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ, pcl::PointXYZ>);
	icp.setInputSource(first_pc);
	icp.setInputTarget(second_pc);
	icp.setTransformationEstimation (trans_lls);
	//pcl::PointCloud<pcl::PointXYZ> Final;
	//icp.align(Final);
	tr_mat_ = icp.getFinalTransformation();
	icp.setMaximumIterations (5);
  	icp.setTransformationEpsilon (1e-8);
  	icp.setMaxCorrespondenceDistance (0.5);


	return tr_mat_;
	}
*/
void PointCloudProcessor::calcICP(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pc (new pcl::PointCloud<pcl::PointXYZ>);
	*src_pc = cloud_seq_loaded[0];
	*tgt_pc = cloud_seq_loaded[1];
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);
	
	pcl::NormalEstimation<pcl::PointXYZ,pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	norm_est.setSearchMethod (tree);
  	norm_est.setKSearch (30);
  	norm_est.setInputCloud (src_pc);
  	norm_est.compute (*points_with_normals_src);
  	pcl::copyPointCloud (*src_pc, *points_with_normals_src);

  	norm_est.setInputCloud (tgt_pc);
  	norm_est.compute (*points_with_normals_tgt);
  	pcl::copyPointCloud (*tgt_pc, *points_with_normals_tgt);
  	 MyPointRepresentation point_representation;
  	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  	point_representation.setRescaleValues (alpha);
  	// Align
  	pcl::IterativeClosestPointNonLinear<pcl::PointNormal,pcl::PointNormal> reg;
  	reg.setTransformationEpsilon (1e-6);
  	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
  	// Note: adjust this based on the size of your datasets
  	reg.setMaxCorrespondenceDistance (0.5);  
  	// Set the point representation
  	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  	reg.setInputSource (points_with_normals_src);
  	reg.setInputTarget (points_with_normals_tgt);
  	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  	pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  	reg.setMaximumIterations (2);
	for (int i = 0; i < 10; ++i)
  	{
    	PCL_INFO ("Iteration Nr. %d.\n", i);
    	// save cloud for visualization purpose
    	points_with_normals_src = reg_result;
    	// Estimate
    	reg.setInputSource (points_with_normals_src);
    	reg.align (*reg_result);
		//accumulate transformation between each Iteration
	    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    	if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      		reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    	prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  	}

  	std::cout<< Ti<< std::endl;


}



/*			
int main(int argc, char **argv)
{	ros::init(argc, argv, "PointCloudrocessor");
	PointCloudProcessor cld_process;

	cld_process.subr();
	ros::spin();
	
	
	return 0;
}*/

