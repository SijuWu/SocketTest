#include "stdafx.h"
#include "KinectOpenNI.h"
#include "LeapListener.h"
#include "PointCloud.h"
#include "PCLHand.h"
#include "PCLFinger.h"
#include "SocketClient.h"
#include "Touch.h"
using namespace PQ_SDK_MultiTouch;


enum DataStruct{Hand=1, Finger=2, Head=3, ScreenTouch=4};

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
//To add touch state
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
//Touch input
Touch touch;

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
	//int err_code=touch.Init();
	//if(err_code != PQMTE_SUCCESS){
	//cout << "press any key to exit..." << endl;
	//getchar();
	//return 0;
	//}
	//// do other things of your application;
	//cout << "hello world" << endl;
	//
	//Client socket
	SocketClient client;
	bool connection=client.ConnectToHost(8000,"192.168.193.200");
	//bool connection=client.ConnectToHost(8000,"192.168.1.2");

	////Create KinectOpenNI engine.
	//KinectOpenNI kinectOpenNI;
	////Run Kinect.
	//xn::SkeletonCapability skeletonCap=kinectOpenNI.KinectRun();
	////Create point cloud viewer.
	//pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
	//Point cloud of Leap data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr leapCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//Create depth image window.
	cvNamedWindow("depth",1);  
	//Create color image window.
	cvNamedWindow("image",1);

	//Add event listener to the controller.
	controller.addListener(leapListener);

	while(key!=27&&connection==true)
	{
		/*std::cout << "Frame id: " << controller.frame().id()
		<< ", timestamp: " << controller.frame().timestamp()
		<< ", hands: " << controller.frame().hands().count()
		<< ", fingers: " << controller.frame().fingers().count()
		<< ", tools: " << controller.frame().tools().count()
		<< ", gestures: " << controller.frame().gestures().count() << std::endl;*/

		////Update kinect for each frame.
		//kinectOpenNI.kinectUpdate();
		////Get depth and color image.
		//kinectOpenNI.getCVImage(&depthImage,&colorImage);
		////Check if any user is detected.
		//userFound=kinectOpenNI.checkUser(&skeletonCap, colorImage);

		frameCount++;

		char headByte[headStructSize];
		char rightHandByte[handStructSize];
		char leftHandByte[handStructSize];
		char handsByte[handStructSize*2];
		char fingersByte[fingerStructSize*10];
		char touchesByte[touchStructSize*10];
		HeadStruct headStruct;
		headStruct.structType=DataStruct::Head;
		headStruct.valid=false;
		HandStruct rightHandStruct;
		rightHandStruct.valid=false;
		rightHandStruct.structType=DataStruct::Hand;
		HandStruct leftHandStruct;
		leftHandStruct.valid=false;
		leftHandStruct.structType=DataStruct::Hand;

		HandStruct handStruct[2];
		for(int i=0;i<2;++i)
		{
			handStruct[i].structType=DataStruct::Hand;
			handStruct[i].valid=false;
			handStruct[i].handId=-1;
			for(int j=0;j<5;++j)
			{
				handStruct[i].fingerIds[j]=-1;
			}
			handStruct[i].frame=frameCount;
			handStruct[i].handPosition.x=-1;
			handStruct[i].handPosition.y=-1;
			handStruct[i].handPosition.z=-1;
			handStruct[i].handOrientation.x=-1;
			handStruct[i].handOrientation.y=-1;
			handStruct[i].handOrientation.z=-1;
		}

		FingerStruct fingerStruct[10];
		for(int i=0;i<10;++i)
		{
			fingerStruct[i].structType=DataStruct::Finger;
			fingerStruct[i].valid=false;
			fingerStruct[i].fingerId=-1;
			fingerStruct[i].handId=-1;
			fingerStruct[i].frame=frameCount;
			fingerStruct[i].fingerPosition.x=-1;
			fingerStruct[i].fingerPosition.y=-1;
			fingerStruct[i].fingerPosition.z=-1;
			fingerStruct[i].fingerOrientation.x=-1;
			fingerStruct[i].fingerOrientation.y=-1;
			fingerStruct[i].fingerOrientation.z=-1;
		}

		for(int i=0;i<controller.frame().fingers().count();++i)
		{
			Leap::Finger finger=controller.frame().fingers()[i];
			fingerStruct[i].valid=true;
			fingerStruct[i].fingerId=finger.id();
			fingerStruct[i].handId=finger.hand().id();
			fingerStruct[i].fingerPosition.x=finger.tipPosition().x;
			fingerStruct[i].fingerPosition.y=finger.tipPosition().y;
			fingerStruct[i].fingerPosition.z=finger.tipPosition().z;
			fingerStruct[i].fingerOrientation.x=finger.direction().x;
			fingerStruct[i].fingerOrientation.y=finger.direction().y;
			fingerStruct[i].fingerOrientation.z=finger.direction().z;
		}

		for(int i=0;i<controller.frame().hands().count();++i)
		{
			Leap::Hand hand=controller.frame().hands()[i];
			handStruct[i].valid=true;
			handStruct[i].handId=hand.id();
			for(int j=0;j<hand.fingers().count();++j)
			{
				handStruct[i].fingerIds[j]=hand.fingers()[j].id();
			}
			handStruct[i].handPosition.x=hand.palmPosition().x;
			handStruct[i].handPosition.y=hand.palmPosition().y;
			handStruct[i].handPosition.z=hand.palmPosition().z;
			handStruct[i].handOrientation.x=hand.direction().x;
			handStruct[i].handOrientation.y=hand.direction().y;
			handStruct[i].handOrientation.z=hand.direction().z;
		}

		for(int i=0;i<controller.frame().fingers().count();++i)
		{
			Leap::Finger finger=controller.frame().fingers()[i];
			fingerStruct[i].valid=true;
			fingerStruct[i].fingerId=finger.id();
			fingerStruct[i].handId=finger.hand().id();
			fingerStruct[i].fingerPosition.x=finger.tipPosition().x;
			fingerStruct[i].fingerPosition.y=finger.tipPosition().y;
			fingerStruct[i].fingerPosition.z=finger.tipPosition().z;
			fingerStruct[i].fingerOrientation.x=finger.direction().x;
			fingerStruct[i].fingerOrientation.y=finger.direction().y;
			fingerStruct[i].fingerOrientation.z=finger.direction().z;
		}

		for(int i=0;i<2;++i)
		{
			char handByte[handStructSize];
			handConversion(handStruct[i],handByte);
			memmove(&handsByte[i*handStructSize],handByte,handStructSize);
		}

		for(int i=0;i<10;++i)
		{
			char fingerByte[fingerStructSize];
			fingerConversion(fingerStruct[i],fingerByte);
			memmove(&fingersByte[i*fingerStructSize],fingerByte,fingerStructSize);
		}

		TouchStruct touchStruct[10];
		for(int i=0;i<10;++i)
		{
			touchStruct[i].structType=DataStruct::ScreenTouch;
			touchStruct[i].valid=false;
			touchStruct[i].touchId=-1;
			touchStruct[i].frame=frameCount;
			touchStruct[i].touchPosition.x=-1;
			touchStruct[i].touchPosition.y=-1;
		}

		std::vector<TouchPoint> pointList=touch.getTouchPointList();

		int resolutionX=touch.getResolutionX();
		int resolutionY=touch.getResolutionY();

		for(int i=0;i<pointList.size();++i)
		{
			touchStruct[i].valid=true;
			touchStruct[i].touchId=pointList[i].id;
			touchStruct[i].touchPosition.x=((float)pointList[i].x/(float)resolutionX);
			touchStruct[i].touchPosition.y=((float)pointList[i].y/(float)resolutionY);
			
		}
		
		for(int i=0;i<10;++i)
		{
			char touchByte[touchStructSize];
			touchConversion(touchStruct[i],touchByte);
			memmove(&touchesByte[i*touchStructSize],touchByte,touchStructSize);
		}
		

	
		/*if(userFound==true)
		{
			headStruct.headId=kinectOpenNI.getHeadId();
			headStruct.frame=frameCount;
			headStruct.headPosition.x=kinectOpenNI.getHead().x;
			headStruct.headPosition.y=kinectOpenNI.getHead().y;
			headStruct.headPosition.z=kinectOpenNI.getHead().z;
			headStruct.valid=true;
			

			rightHandStruct.handId=kinectOpenNI.getRightHandId();
			leftHandStruct.handId=kinectOpenNI.getLeftHandId();
			rightHandStruct.frame=frameCount;
			leftHandStruct.frame=frameCount;
			rightHandStruct.handPosition.x=kinectOpenNI.getRightHand().x;
			rightHandStruct.handPosition.y=kinectOpenNI.getRightHand().y;
			rightHandStruct.handPosition.z=kinectOpenNI.getRightHand().z;
			leftHandStruct.handPosition.x=kinectOpenNI.getLeftHand().x;
			leftHandStruct.handPosition.y=kinectOpenNI.getLeftHand().y;
			leftHandStruct.handPosition.z=kinectOpenNI.getLeftHand().z;
			rightHandStruct.valid=true;
			leftHandStruct.valid=true;
			
		}*/
		headConversion(headStruct,headByte);
		handConversion(rightHandStruct,rightHandByte);
		handConversion(leftHandStruct,leftHandByte);

		char dataToSend[headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize];
		memmove(dataToSend,headByte,headStructSize);
		///////////////////Kinect hands
		/*memmove(&dataToSend[headStructSize],rightHandByte,handStructSize);
		memmove(&dataToSend[headStructSize+handStructSize],leftHandByte,handStructSize);*/
		///////////////////
		///////////////////Leap hands
		memmove(&dataToSend[headStructSize],handsByte,2*handStructSize);
		///////////////////
		memmove(&dataToSend[headStructSize+2*handStructSize],touchesByte,touchStructSize*10);
		memmove(&dataToSend[headStructSize+2*handStructSize+10*touchStructSize],fingersByte,fingerStructSize*10);
		send(client.getClientSocket(),dataToSend,headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize,0);
		

		//Create point cloud for the environment.
		//pointCloud.createCloudXYZ(kinectOpenNI.getDepthData());
		//Display the point cloud.


		/*pclHands.resize(0);
		pclFingers.resize(0);*/


		/*HandStruct testHand;
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
		touchConversion(testTouch,touchByte);*/

		/*char dataToSend[156];
		memmove(dataToSend,handByte,57);
		memmove(&dataToSend[57],fingerByte,41);
		memmove(&dataToSend[98],headByte,37);
		memmove(&dataToSend[135],touchByte,21);*/
		//send(client.getClientSocket(),dataToSend,156,0);







		for(int i=0;i<controller.frame().hands().count();++i)
		{
			PCLHand hand=PCLHand(&controller.frame().hands()[i]);
			pclHands.push_back(&hand);
			//std::cout<<controller.frame().hands()[i].palmPosition();
			//send(client.getClientSocket(),"Hand",strlen("Hand"),0);
		}
		for(int i=0;i<controller.frame().fingers().count();++i)
		{
			PCLFinger finger=PCLFinger(&controller.frame().fingers()[i]);
			pclFingers.push_back(&finger);
		}

		if(controller.frame().hands().count()!=0&&controller.frame().fingers().count()!=0)
		{
			std::cout<<"Hand and finger"<<std::endl;
		}

		if(controller.frame().hands().count()!=0&&controller.frame().fingers().count()==0)
		{
			std::cout<<"Hand"<<std::endl;
		}

		if(controller.frame().hands().count()==0&&controller.frame().fingers().count()!=0)
		{
			std::cout<<"Finger"<<std::endl;
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



		//cloudViewer.showCloud(pointCloud.getCloudXYZ());

		//Display depth and color image.
		/*cv::imshow("depth",depthImage);
		cv::imshow("image",colorImage);*/

		key=cv::waitKey(20);

	}
	//Close the kinect engine.
	//kinectOpenNI.KinectClose();
	return 0;  
}  


