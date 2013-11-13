#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <Eigen/StdVector>
#include <stdlib.h>  
#include "SplitCloud2.h"

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include "opencv/cv.h"  

class PointCloud
{
public:
	enum Hand{LeftHand=0,RightHand=1};
	PointCloud(void);
	~PointCloud(void);

	//Create the point cloud without color.
	void createCloudXYZ( const XnDepthPixel* pDepth);
	void createCloudXYZ(cv::Mat* depthImage);

	//Create the point cloud with color.
	void createCloudXYZRGBA( const XnDepthPixel* pDepth,const XnUInt8* pColor);
	void createCloudXYZRGBA(cv::Mat* depthImage,const XnUInt8* pColor);

	//Get the point cloud without color.
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudXYZ();
	//Get the point cloud with color.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloudXYZRGBA();
	
	//Get the downsampling of the point loud
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,float xLeafSize,float yLeafSize, float zLeafSize);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,float xLeafSize,float yLeafSize, float zLeafSize);
	
	//Filter the points outside the limit
	pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,std::string axis,float minLimit, float maxLimit);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,std::string axis,float minLimit, float maxLimit);
	
	//Find a plane from the point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloudPlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource);

	//Find the concave hull of the plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPlaneConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,double distanceThreshold,double alpha);
	//Find the convex hull of the plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPlaneConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,double distanceThreshold);
	
	//Extract clusters from the point cloud
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,double tolerance,int minClusterSize,int maxClusterSize);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,double tolerance,int minClusterSize,int maxClusterSize);

	//Find neighbours in the octree voxel
	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourOctreeVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourOctreeVoxel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution, pcl::PointXYZRGBA* searchPoint);

	//Find K closest neighbours by using octree 
	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourOctreeKNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,int neighbourNum, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourOctreeKNeighbour(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution,int neighbourNum, pcl::PointXYZRGBA* searchPoint);

	//Find neighbours within the radius by using octree
	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourOctreeRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourOctreeRadius(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZRGBA* searchPoint);
	//Find neighbours without the radius by using octree
	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourOctreeOutsideRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZ* searchPoint);

	//Find K closest neighbours by using KdTree
	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourKdTreeKNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,int neighbourNum, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourKdTreeKNeighbour(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,int neighbourNum, pcl::PointXYZRGBA* searchPoint);

	//Find neighbours within the radius by using kdTree
	pcl::PointCloud<pcl::PointXYZ>::Ptr searchNeighbourKdTreeRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float radius, pcl::PointXYZ* searchPoint);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchNeighbourKdTreeRadius(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float radius, pcl::PointXYZRGBA* searchPoint);

	//Get the potential hand from the point cloud
	bool getNearBlobs2( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr leftHandCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr rightHandCloud);
	bool getNearBlobs2( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leftHandCloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rightHandCloud/*, std::vector<Eigen::Vector4f> &nearcents*/);
	bool getNearBlobs2( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud);
	bool getNearBlobs2( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newCloud);

	//Get the point cloud in which points are adjoints of cloudpts
	bool findNearbyPts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid);
	bool findNearbyPts(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid);

	//Get the sub point cloud
	void getSubCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud);
	void getSubCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr subCloud);
	//If the keep is true, get the sub point cloud, else get the point cloud which is filtered
	void getSubCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud,bool keep);

	//Get neighbours of center point within the radius
	void NNN(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, double radius);
	void NNN(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, std::vector<float> &dists, double radius);

	void NNN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* center, std::vector<int> &inds, double radius);
	void NNN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* center, std::vector<int> &inds, std::vector<float> &dists, double radius);

	void NNN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, double radius);
	void NNN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, std::vector<float> &dists, double radius);

	//Get the direction of the hand
	void getEigens(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,int hand);//0Left 1 Right
	//Check if the direction of the hand is inverse
	void PointCloud::flipvec(const Eigen::Vector4f &palm, const Eigen::Vector4f &fcentroid,Eigen::Vector4f &dir );
    //Divide the hand point cloud into a palm cloud and a rigits cloud by using density
	void radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,int nnthresh,double tol,int hand,pcl::PointCloud<pcl::PointXYZ>::Ptr palm,pcl::PointCloud<pcl::PointXYZ>::Ptr digits);
	void radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,int resolution,int radius,int hand,pcl::PointCloud<pcl::PointXYZ>::Ptr digits,pcl::PointXYZ* center);
	//Divide the hand point cloud into a palm cloud and a rigits cloud by checking the covariance
	void covarianceFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,double tol,int hand, pcl::PointCloud<pcl::PointXYZ>::Ptr palm, pcl::PointCloud<pcl::PointXYZ>::Ptr digits);
	void covarianceFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,double tol,int hand, float resolution,float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr palm, pcl::PointCloud<pcl::PointXYZ>::Ptr digits);
	//Divide the hand point cloud into a palm cloud and a rigits cloud by checking normal values
	void normalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,int hand, int radius,pcl::PointCloud<pcl::PointXYZ>::Ptr digits);
	
	//Extract fingers from the rigits cloud
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segFingers(pcl::PointCloud<pcl::PointXYZ>::Ptr digits,double clustertol,int mincluster);
	
	//Create a color point cloud from a point cloud without color
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getColorPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr initialHand,int red,int green,int blue);

	//Check the angle of fingers
	double checkFingerAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr fingerCloud);

	//Check the distance of the finger tip to the hand center
	double checkFingerDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr fingerCloud,Eigen::Vector3f* fingerDirection,int hand);
	
	//Get the finger direction
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getFingerLine(pcl::PointCloud<pcl::PointXYZ>::Ptr fingerCloud);
	
	//Get the hand center
	Eigen::Vector4f getHandCenter(int hand);
	
	//Get the hand direction
	Eigen::Vector4f getHandDirection();
	
	//Set the center of the arm
	void setArmCenter(pcl::PointXYZ* armCenter,int hand);

	void computeEigenVector(pcl::PointCloud<pcl::PointXYZ>* pointCloud, Eigen::Vector4f* centroid,Eigen::Matrix3f* eigen_vectors);
private:
	//Point cloud without color
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
	//Point cloud with color
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA;
	//Arm centers
	Eigen::Vector4f arm_center[2];
	//Hand points
	Eigen::Vector4f handPoints[2];
	//Hand direction
	Eigen::Vector4f handDirection;
	//Distance from the sensor
	double distfromsensor;

	


};

