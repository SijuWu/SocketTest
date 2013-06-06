#pragma once

#include <stdlib.h>  
#include <iostream>  
#include <string>  
#include <XnCppWrapper.h>
#include "opencv/cv.h"  
#include "opencv/highgui.h" 
#include "PointCloud.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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
	bool checkUser(xn::SkeletonCapability* skeletonCap);
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
	 
};

