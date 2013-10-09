#include "stdafx.h"
#include "KinectOpenNI.h"
#include "LeapListener.h"
#include "PointCloud.h"
#include "PCLHand.h"
#include "PCLFinger.h"
#include "SocketClient.h"

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
//Leap motion event listener;
LeapListener leapListener;
//Leap motion controller;
Controller controller;
//Vector of handPoint
std::vector<PCLHand*> pclHands;
//Vector of fingerPoint
std::vector<PCLFinger*> pclFingers;

int main( int argc, char** argv )  
{  
	//Client socket
	SocketClient client;
	bool connection=client.ConnectToHost(8000,"192.168.193.200");


	//Create KinectOpenNI engine.
	KinectOpenNI kinectOpenNI;
	//Run Kinect.
	xn::SkeletonCapability skeletonCap=kinectOpenNI.KinectRun();
	//Create point cloud viewer.
	pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
	//Point cloud of Leap data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr leapCloud(new pcl::PointCloud<pcl::PointXYZRGB>);




	//Create depth image window.
	cvNamedWindow("depth",1);  
	//Create color image window.
	cvNamedWindow("image",1);

	////Add event listener to the controller.
	//controller.addListener(leapListener);

	while(key!=27&&connection==true)
	{
		/*std::cout << "Frame id: " << controller.frame().id()
		<< ", timestamp: " << controller.frame().timestamp()
		<< ", hands: " << controller.frame().hands().count()
		<< ", fingers: " << controller.frame().fingers().count()
		<< ", tools: " << controller.frame().tools().count()
		<< ", gestures: " << controller.frame().gestures().count() << std::endl;*/

		//Update kinect for each frame.
		kinectOpenNI.kinectUpdate();
		//Get depth and color image.
		kinectOpenNI.getCVImage(&depthImage,&colorImage);
		//Check if any user is detected.
		userFound=kinectOpenNI.checkUser(&skeletonCap, colorImage);

		//Create point cloud for the environment.
		pointCloud.createCloudXYZ(kinectOpenNI.getDepthData());
		//Display the point cloud.


		pclHands.resize(0);
		pclFingers.resize(0);

		for(int i=0;i<controller.frame().hands().count();++i)
		{
			PCLHand hand=PCLHand(&controller.frame().hands()[i]);
			pclHands.push_back(&hand);

			send(client.getClientSocket(),"Hand",strlen("Hand"),0);
		}
		for(int i=0;i<controller.frame().fingers().count();++i)
		{
			PCLFinger finger=PCLFinger(&controller.frame().fingers()[i]);
			pclFingers.push_back(&finger);
		}
	
		

		leapCloud->points.resize(0);

		for(int i=0;i<pclHands.size();++i)
		{
			leapCloud->points.push_back(*pclHands.at(i));
		}

		for(int i=0;i<pclFingers.size();++i)
		{
			leapCloud->points.push_back(*pclFingers.at(i));
		}



		cloudViewer.showCloud(pointCloud.getCloudXYZ());

		//Display depth and color image.
		cv::imshow("depth",depthImage);
		cv::imshow("image",colorImage);

		key=cv::waitKey(20);

	}
	//Close the kinect engine.
	kinectOpenNI.KinectClose();
	return 0;  
}  


