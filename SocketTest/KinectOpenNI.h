#pragma once

#include <stdlib.h>  
#include <iostream>  
#include <string>  
#include <XnCppWrapper.h>
#include "opencv/cv.h"  
#include "opencv/highgui.h" 
#include "PointCloud.h"
#include <pcl/visualization/cloud_viewer.h>

#include<pcl/surface/concave_hull.h>
#include <pcl/filters/project_inliers.h>
class KinectOpenNI
{
public:
	KinectOpenNI(void);
	~KinectOpenNI(void);
	xn::SkeletonCapability KinectRun();
	void KinectClose();
	void getCVImage(cv::Mat* depthImage,cv::Mat* colorImage);
	const XnDepthPixel* getDepthData();
	const XnUInt8* getImageData();  
	bool checkUser(xn::SkeletonCapability* skeletonCap, cv::Mat colorImage);
	XnUserID getUserID();
	int getHeadId();
	int getRightHandId();
	int getLeftHandId();
	pcl::PointXYZ getHead();
	pcl::PointXYZ getRightHand();
	pcl::PointXYZ getLeftHand();
	//void displayImage();
	void kinectUpdate();
private:
	void CheckOpenNIError( XnStatus result, std::string status ); 
	
	//bool checkUser(xn::SkeletonCapability* skeletonCap, pcl::PointXYZRGBA* rightHand);
	
	XnStatus result;  
	xn::Context context;   
	//Depth image data
    xn::DepthMetaData depthMD;  
	//Color image data
    xn::ImageMetaData imageMD;  
	//Depth data generator
	xn::DepthGenerator depthGenerator; 
	//Color data generator
	xn::ImageGenerator imageGenerator;  
	//User data generator
	xn::UserGenerator userGenerator;
	//Output image mode
	XnMapOutputMode mapMode;  

	XnCallbackHandle calibCBHandle;
	XnCallbackHandle poseCBHandle; 
	
	//PointCloud pointCloud;

//	//Matrix of depth image
//cv::Mat depthImage;
////Matrix of color image
//cv::Mat colorImage;

	//char key;
    int mode;

	/*pcl::PointXYZRGBA* head;
	pcl::PointXYZRGBA* leftHand;
	pcl::PointXYZRGBA* rightHand;*/
	/*pcl::PointXYZRGBA head;
	pcl::PointXYZRGBA leftHand;
	pcl::PointXYZRGBA rightHand;*/
	//XnUserID* userIDList;
	XnUserID principalUserId;
	int headId;
	int rightHandId;
	int leftHandId;
	pcl::PointXYZ head;
	pcl::PointXYZ headOrientation;
	pcl::PointXYZ leftHand;
	pcl::PointXYZ leftHandOrientation;
	pcl::PointXYZ rightHand;
	pcl::PointXYZ rightHandOrientation;
	pcl::PointXYZ leftElbow;
	pcl::PointXYZ rightElbow;

	double rightHandX;
	double rightHandY;
	double rightHandZ;

	
};

