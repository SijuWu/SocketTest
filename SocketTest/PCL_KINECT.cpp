#include "stdafx.h"
#include "KinectOpenNI.h"
#include "LeapListener.h"
#include "PointCloud.h"
#include "PCLHand.h"
#include "PCLFinger.h"
#include "SocketClient.h"
//#include "Touch.h"


//using namespace PQ_SDK_MultiTouch;


enum DataStruct{Hand=1, Finger=2, Head=3, Touch=4};

//2D position structure, 8 bytes
struct Position2D
{
	float x;
	float y;
};

//3D position structure, 12 bytes
struct Position3D
{
	float x;
	float y;
	float z;
};

//Orientation structure, 12 bytes
struct Orientation
{
	float x;
	float y;
	float z;
};

//Hand data structure, 57 bytes
struct HandStruct
{
public:
	DataStruct structType;
	int handId;
	int fingerIds[5];
	int frame;
	Position3D handPosition;
	Orientation handOrientation;
	bool valid;
};

//Finger data structure, 41 bytes
struct FingerStruct
{
public:
	DataStruct structType;
	int fingerId;
	int handId;
	int frame;
	Position3D fingerPosition;
	Orientation fingerOrientation;
	bool valid;
};

//Head data structure, 37 bytes
struct HeadStruct
{
public:
	DataStruct structType;
	int headId;
	int frame;
	Position3D headPosition;
	Orientation headOrientation;
	bool valid;
};

//Touch data structure, 21 bytes
struct TouchStruct
{
public:
	DataStruct structType;
	int touchId;
	int frame;
	Position2D touchPosition;
	bool valid;
};

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
////Touch input
//Touch touch;

int frameCount=0;
const int handStructSize=57;
const int fingerStructSize=41;
const int headStructSize=37;
const int touchStructSize=21;

void handConversion(HandStruct handStruct, char * handByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&handStruct.structType);
	//Bytes of handId 
	char* handIdByte=reinterpret_cast<char *>(&handStruct.handId);
	//Bytes of fingerIds
	char fingerIdsByte[20];
	for(int i=0;i<5;++i)
	{
		char* fingerIdByte=reinterpret_cast<char *>(&handStruct.fingerIds[i]);
		memmove(&fingerIdsByte[i*4],fingerIdByte,4);
	}
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(&handStruct.frame);
	//Bytes of hand position
	char* positionXByte=reinterpret_cast<char *>(&handStruct.handPosition.x);
	char* positionYByte=reinterpret_cast<char *>(&handStruct.handPosition.y);
	char* positionZByte=reinterpret_cast<char *>(&handStruct.handPosition.z);
	//Bytes of hand orientation
	char* orientationXByte=reinterpret_cast<char *>(&handStruct.handOrientation.x);
	char* orientationYByte=reinterpret_cast<char *>(&handStruct.handOrientation.y);
	char* orientationZByte=reinterpret_cast<char *>(&handStruct.handOrientation.z);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&handStruct.valid);

	memmove(handByte,typeByte,4);
	memmove(&handByte[4],handIdByte,4);
	memmove(&handByte[8],fingerIdsByte,20);
	memmove(&handByte[28],frameByte,4);
	memmove(&handByte[32],positionXByte,4);
	memmove(&handByte[36],positionYByte,4);
	memmove(&handByte[40],positionZByte,4);
	memmove(&handByte[44],orientationXByte,4);
	memmove(&handByte[48],orientationYByte,4);
	memmove(&handByte[52],orientationZByte,4);
	memmove(&handByte[56],validByte,1);
}

void touchConversion(TouchStruct touchStruct, char * touchByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&touchStruct.structType);
	//Bytes of touchId
	char* touchIdByte=reinterpret_cast<char *>(&touchStruct.touchId);
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(&touchStruct.frame);
	//Bytes of touch position
	char* positionXByte=reinterpret_cast<char *>(&touchStruct.touchPosition.x);
	char* positionYByte=reinterpret_cast<char *>(&touchStruct.touchPosition.y);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&touchStruct.valid);

	memmove(touchByte,typeByte,4);
	memmove(&touchByte[4],touchIdByte,4);
	memmove(&touchByte[8],frameByte,4);
	memmove(&touchByte[12],positionXByte,4);
	memmove(&touchByte[16],positionYByte,4);
	memmove(&touchByte[20],validByte,1);
}

void fingerConversion(FingerStruct fingerStruct, char * fingerByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&fingerStruct.structType);
	//Bytes of fingerId
	char* fingerIdByte=reinterpret_cast<char *>(&fingerStruct.fingerId);
	//Bytes of handId
	char* handIdByte=reinterpret_cast<char *>(&fingerStruct.handId);
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(&fingerStruct.frame);
	//Bytes finger position
	char* positionXByte=reinterpret_cast<char *>(&fingerStruct.fingerPosition.x);
	char* positionYByte=reinterpret_cast<char *>(&fingerStruct.fingerPosition.y);
	char* positionZByte=reinterpret_cast<char *>(&fingerStruct.fingerPosition.z);
	//Bytes finger orientation
	char* orientationXByte=reinterpret_cast<char *>(&fingerStruct.fingerOrientation.x);
	char* orientationYByte=reinterpret_cast<char *>(&fingerStruct.fingerOrientation.y);
	char* orientationZByte=reinterpret_cast<char *>(&fingerStruct.fingerOrientation.z);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&fingerStruct.valid);

	memmove(fingerByte,typeByte,4);
	memmove(&fingerByte[4],fingerIdByte,4);
	memmove(&fingerByte[8],handIdByte,4);
	memmove(&fingerByte[12],frameByte,4);
	memmove(&fingerByte[16],positionXByte,4);
	memmove(&fingerByte[20],positionYByte,4);
	memmove(&fingerByte[24],positionZByte,4);
	memmove(&fingerByte[28],orientationXByte,4);
	memmove(&fingerByte[32],orientationYByte,4);
	memmove(&fingerByte[36],orientationZByte,4);
	memmove(&fingerByte[40],validByte,1);
}

void headConversion(HeadStruct headStruct, char * headByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&headStruct.structType);
	//Bytes of headId
	char* headIdByte=reinterpret_cast<char *>(&headStruct.headId);
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(&headStruct.frame);
	//Bytes of head position
	char* positionXByte=reinterpret_cast<char *>(&headStruct.headPosition.x);
	char* positionYByte=reinterpret_cast<char *>(&headStruct.headPosition.y);
	char* positionZByte=reinterpret_cast<char *>(&headStruct.headPosition.z);
	//Bytes of head orientation
	char* orientationXByte=reinterpret_cast<char *>(&headStruct.headOrientation.x);
	char* orientationYByte=reinterpret_cast<char *>(&headStruct.headOrientation.y);
	char* orientationZByte=reinterpret_cast<char *>(&headStruct.headOrientation.z);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&headStruct.valid);

	memmove(headByte,typeByte,4);
	memmove(&headByte[4],headIdByte,4);
	memmove(&headByte[8],frameByte,4);
	memmove(&headByte[12],positionXByte,4);
	memmove(&headByte[16],positionYByte,4);
	memmove(&headByte[20],positionZByte,4);
	memmove(&headByte[24],orientationXByte,4);
	memmove(&headByte[28],orientationYByte,4);
	memmove(&headByte[32],orientationZByte,4);
	memmove(&headByte[36],validByte,1);
}

int main( int argc, char** argv )  
{  
	/*int err_code=touch.Init();
	if(err_code != PQMTE_SUCCESS){
	cout << "press any key to exit..." << endl;
	getchar();
	return 0;
	}*/
	// do other things of your application;
	cout << "hello world" << endl;
	//
	//Client socket
	SocketClient client;
	//bool connection=client.ConnectToHost(8000,"192.168.193.200");
	bool connection=client.ConnectToHost(8000,"192.168.1.5");

	//Create KinectOpenNI engine.
	KinectOpenNI kinectOpenNI;
	//Run Kinect.
	xn::SkeletonCapability skeletonCap=kinectOpenNI.KinectRun();
	////Create point cloud viewer.
	//pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
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
		//pointCloud.createCloudXYZ(kinectOpenNI.getDepthData());
		//Display the point cloud.


		/*pclHands.resize(0);
		pclFingers.resize(0);*/
		frameCount++;

		HandStruct testHand;
		testHand.structType=DataStruct::Hand;
		testHand.handId=1;
		for(int i=0;i<5;++i)
		{
			testHand.fingerIds[i]=i;
		}
		testHand.frame=frameCount;
		testHand.handPosition.x=1.1;
		testHand.handPosition.y=2.2;
		testHand.handPosition.z=3.3;
		testHand.handOrientation.x=1.1;
		testHand.handOrientation.y=2.2;
		testHand.handOrientation.z=3.3;
		testHand.valid=true;
		char handByte[57];
		handConversion(testHand,handByte);

		FingerStruct testFinger;
		testFinger.structType=DataStruct::Finger;
		testFinger.fingerId=2;
		testFinger.handId=1;
		testFinger.frame=frameCount;
		testFinger.fingerPosition.x=4.4;
		testFinger.fingerPosition.y=5.5;
		testFinger.fingerPosition.z=6.6;
		testFinger.fingerOrientation.x=7.7;
		testFinger.fingerOrientation.y=8.8;
		testFinger.fingerOrientation.z=9.9;
		testFinger.valid=true;
		char fingerByte[41];
		fingerConversion(testFinger,fingerByte);

		HeadStruct testHead;
		testHead.structType=DataStruct::Head;
		testHead.headId=3;
		testHead.frame=frameCount;
		testHead.headPosition.x=-4.4;
		testHead.headPosition.y=-5.5;
		testHead.headPosition.z=-6.6;
		testHead.headOrientation.x=-7.7;
		testHead.headOrientation.y=-8.8;
		testHead.headOrientation.z=-9.9;
		testHead.valid=true;
		char headByte[37];
		headConversion(testHead,headByte);

		TouchStruct testTouch;
		testTouch.structType=DataStruct::Touch;
		testTouch.touchId=3;
		testTouch.frame=frameCount;
		testTouch.touchPosition.x=-1.1;
		testTouch.touchPosition.y=-2.2;
		testTouch.valid=true;
		char touchByte[21];
		touchConversion(testTouch,touchByte);

		char dataToSend[156];
		memmove(dataToSend,handByte,57);
		memmove(&dataToSend[57],fingerByte,41);
		memmove(&dataToSend[98],headByte,37);
		memmove(&dataToSend[135],touchByte,21);
		send(client.getClientSocket(),dataToSend,156,0);
		
		
		

	


		//for(int i=0;i<controller.frame().hands().count();++i)
		//{
		//	PCLHand hand=PCLHand(&controller.frame().hands()[i]);
		//	pclHands.push_back(&hand);
		//	//std::cout<<controller.frame().hands()[i].palmPosition();
		//	//send(client.getClientSocket(),"Hand",strlen("Hand"),0);
		//}
		//for(int i=0;i<controller.frame().fingers().count();++i)
		//{
		//	PCLFinger finger=PCLFinger(&controller.frame().fingers()[i]);
		//	pclFingers.push_back(&finger);
		//}

		//if(controller.frame().hands().count()!=0&&controller.frame().fingers().count()!=0)
		//{
		//	//send(client.getClientSocket(),"Hand and finger",strlen("Hand and finger"),0);
		//	std::cout<<"Hand and finger"<<std::endl;
		//}

		//if(controller.frame().hands().count()!=0&&controller.frame().fingers().count()==0)
		//{
		//	//send(client.getClientSocket(),"Hand",strlen("Hand"),0);
		//	std::cout<<"Hand"<<std::endl;
		//}

		//if(controller.frame().hands().count()==0&&controller.frame().fingers().count()!=0)
		//{
		//	//send(client.getClientSocket(),"Finger",strlen("Finger"),0);
		//	std::cout<<"Finger"<<std::endl;
		//}

		//if(controller.frame().hands().count()==0&&controller.frame().fingers().count()==0)
		//{
		//	//send(client.getClientSocket(),"None",strlen("None"),0);
		//	std::cout<<"None"<<std::endl;
		//}

		//leapCloud->points.resize(0);

		//for(int i=0;i<pclHands.size();++i)
		//{
		//	leapCloud->points.push_back(*pclHands.at(i));
		//}

		//for(int i=0;i<pclFingers.size();++i)
		//{
		//	leapCloud->points.push_back(*pclFingers.at(i));
		//}



		//cloudViewer.showCloud(pointCloud.getCloudXYZ());

		//Display depth and color image.
		cv::imshow("depth",depthImage);
		cv::imshow("image",colorImage);

		key=cv::waitKey(20);

	}
	//Close the kinect engine.
	kinectOpenNI.KinectClose();
	return 0;  
}  


