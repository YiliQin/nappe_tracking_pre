/* \author Yili Qin
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>

#include <iostream>
#include <chrono>
#include <string>

#define OUTPUT_TIME_INFO true
#define OUTPUT_DEBUG_INFO false
#define VOXEL_LEAFSIZE 0.04

ros::Publisher voxel_pub;
int cntRun = 0;

/**  Voxel filter. */
pcl::PointCloud<pcl::PointXYZRGB> * voxel_filter(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrTmp(&cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cptrCloud(ptrTmp);
	pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudVoxel = new (pcl::PointCloud<pcl::PointXYZRGB>);

	auto tic_voxel = std::chrono::high_resolution_clock::now();

	vg.setInputCloud(cptrCloud);
	// For nappe
	vg.setLeafSize(VOXEL_LEAFSIZE, VOXEL_LEAFSIZE, VOXEL_LEAFSIZE);
	vg.filter(*ptrCloudVoxel);

	auto toc_voxel = std::chrono::high_resolution_clock::now();
	std::chrono::microseconds dur_ms;
	std::chrono::duration<double, std::milli> dur_voxel_ms = toc_voxel - tic_voxel; 
	if (OUTPUT_TIME_INFO == true)
			std::cout << "Voxel filter duration(ms) >>> " << dur_voxel_ms.count() << std::endl;

	return ptrCloudVoxel;
}

/** Tracking the surface of the deformable object. */
void nappe_tracking_pre(const sensor_msgs::PointCloud2ConstPtr & input)
{
	cntRun++;
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "Callback running ... " << cntRun << std::endl;

  // Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
  pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudRGB = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::fromROSMsg(*input, *ptrCloudRGB);

  // Voxel filter
  pcl::PointCloud<pcl::PointXYZRGB> * ptrVoxelFilter = new pcl::PointCloud<pcl::PointXYZRGB>;
  ptrVoxelFilter = voxel_filter(*ptrCloudRGB);

  // Publish topic
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*ptrVoxelFilter, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "camera_depth_optical_frame";
  voxel_pub.publish(output);
}

int main(int argc, char * argv[])
{
	// System info
	std::cout << "PCL Version: " << PCL_VERSION << std::endl;
	std::cout << "Voxel flter(leaf): " << VOXEL_LEAFSIZE << std::endl;
	std::cout << "Nappe Pre-process running ...  " << std::endl;

	// Initialize ROS
	ros::init(argc, argv, "nappe_tracking_pre");
	ros::NodeHandle nh;
	
	// ROS subscriber & publisher 
	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, nappe_tracking_pre);
	voxel_pub = nh.advertise<sensor_msgs::PointCloud2>("/nappe/filter/voxel", 1);

	// Spin
	ros::spin();
	
	return 0;
}
