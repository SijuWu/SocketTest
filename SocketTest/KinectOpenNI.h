#pragma once

#include <stdlib.h>  
#include <iostream>  
#include <string>  
#include <XnCppWrapper.h>
#include "opencv/cv.h"  
#include "opencv/highgui.h" 
#include "PointCloud.h"
#include <pcl/visualization/cloud_viewer.h>

class KinectOpenNI
{
public:
	KinectOpenNI(void);
	~KinectOpenNI(void);
	void KinectRun();
	void KinectClose();


private:
	void CheckOpenNIError( XnStatus result, std::string status ); 
	void getCVImage(cv::Mat* depthImage,cv::Mat* colorImage);
	void displayImage();
	//bool checkUser(xn::SkeletonCapability* skeletonCap, pcl::PointXYZRGBA* rightHand);
	bool checkUser(xn::SkeletonCapability* skeletonCap/*, pcl::PointXYZRGBA* rightHand*/);
	XnStatus result;  
	xn::Context context;   
    xn::DepthMetaData depthMD;  
    xn::ImageMetaData imageMD;  
	xn::DepthGenerator depthGenerator; 
	xn::ImageGenerator imageGenerator;  
	xn::UserGenerator userGenerator;
	XnMapOutputMode mapMode;  
	//xn::SkeletonCapability skeletonCap=NULL;
	XnCallbackHandle calibCBHandle;
	XnCallbackHandle poseCBHandle; 
	
	PointCloud pointCloud;
	cv::Mat depthImage;
	cv::Mat colorImage;
	char key;
    int mode;

	/*pcl::PointXYZRGBA* head;
	pcl::PointXYZRGBA* leftHand;
	pcl::PointXYZRGBA* rightHand;*/
	/*pcl::PointXYZRGBA head;
	pcl::PointXYZRGBA leftHand;
	pcl::PointXYZRGBA rightHand;*/

	pcl::PointXYZ head;
	pcl::PointXYZ leftHand;
	pcl::PointXYZ rightHand;

	double rightHandX;
	double rightHandY;
	double rightHandZ;

	
};

