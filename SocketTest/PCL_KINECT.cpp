#include "stdafx.h"
#include "KinectOpenNI.h"

#include "PointCloud.h"
//Keyboard input
char key=0;
//Matrix of depth image
cv::Mat depthImage;
//Matrix of color image
cv::Mat colorImage;
//PointCloud
PointCloud pointCloud;
//Check if any user is found
bool userFound;

int main( int argc, char** argv )  
{  

	KinectOpenNI kinectOpenNI;
	xn::SkeletonCapability skeletonCap=kinectOpenNI.KinectRun();
	kinectOpenNI.KinectRun();
	pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
	cvNamedWindow("depth",1);  
	cvNamedWindow("image",1);
	while(key!=27)
	{
		kinectOpenNI.kinectUpdate();
		kinectOpenNI.getCVImage(&depthImage,&colorImage);

		userFound=kinectOpenNI.checkUser(&skeletonCap, colorImage);

		pointCloud.createCloudXYZ(kinectOpenNI.getDepthData());
		cloudViewer.showCloud(pointCloud.getCloudXYZ());

		

		cv::imshow("depth",depthImage);
		cv::imshow("image",colorImage);

		key=cv::waitKey(20);

	}
	kinectOpenNI.KinectClose();
	return 0;  
}  


