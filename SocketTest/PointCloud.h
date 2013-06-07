#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include <XnCppWrapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>


class PointCloud
{
public:
	PointCloud(void);
	~PointCloud(void);
	void createCloudXYZ( const XnDepthPixel* pDepth);
	void createCloudXYZRGBA( const XnDepthPixel* pDepth,const XnUInt8* pColor);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudXYZ();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloudXYZRGBA();
	
	/*template<typename PointT> pcl::PointCloud<PointT>::Ptr downSampling(pcl::PointCloud<PointT>::Ptr cloudSource,float xLeafSize,float yLeafSize, float zLeafSize);*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,float xLeafSize,float yLeafSize, float zLeafSize);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,float xLeafSize,float yLeafSize, float zLeafSize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,std::string axis,float minLimit, float maxLimit);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,std::string axis,float minLimit, float maxLimit);
	//pcl::RangeImage getRangeImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ);
	//pcl::RangeImage getRangeImage(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloudPlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA;
	


};

