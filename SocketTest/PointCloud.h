#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include <XnCppWrapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/range_image/range_image.h>
class PointCloud
{
public:
	PointCloud(void);
	~PointCloud(void);
	void createCloudXYZ( const XnDepthPixel* pDepth);
	void createCloudXYZRGBA( const XnDepthPixel* pDepth,const XnUInt8* pColor);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudXYZ();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloudXYZRGBA();
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,float xLeafSize,float yLeafSize, float zLeafSize);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,float xLeafSize,float yLeafSize, float zLeafSize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,std::string axis,float minLimit, float maxLimit);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,std::string axis,float minLimit, float maxLimit);
	pcl::RangeImage getRangeImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ);
	pcl::RangeImage getRangeImage(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA);
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA;
	float angularResolution;
	float maxAngleWidth;
	float maxAngleHeight;


};
