#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/octree/octree.h>
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
#include <Eigen/StdVector>
#include <stdlib.h>  
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourOctreeVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourOctreeVoxel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution, pcl::PointXYZRGBA* searchPoint);

	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourOctreeKNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,int neighbourNum, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourOctreeKNeighbour(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution,int neighbourNum, pcl::PointXYZRGBA* searchPoint);

	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourOctreeRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourOctreeRadius(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZRGBA* searchPoint);


	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourKdTreeKNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,int neighbourNum, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourKdTreeKNeighbour(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,int neighbourNum, pcl::PointXYZRGBA* searchPoint);

	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourKdTreeRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float radius, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourKdTreeRadius(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float radius, pcl::PointXYZRGBA* searchPoint);

	
	bool getNearBlobs2( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr leftHandCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr rightHandCloud/*, std::vector<Eigen::Vector4f> &nearcents*/);
	bool getNearBlobs2( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leftHandCloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rightHandCloud/*, std::vector<Eigen::Vector4f> &nearcents*/);
	
	bool findNearbyPts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid);
	bool findNearbyPts(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid);

	void getSubCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZRGBA> &subCloud);

	void getSubCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud);
	void getSubCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr subCloud);



	void NNN(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, double radius);
	void NNN(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, std::vector<float> &dists, double radius);



	void NNN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* center, std::vector<int> &inds, double radius);
	void NNN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* center, std::vector<int> &inds, std::vector<float> &dists, double radius);

	void NNN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, double radius);
	void NNN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, std::vector<float> &dists, double radius);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA;



};

