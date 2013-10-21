#include "stdafx.h"
#include "KinectOpenNI.h"
#include "LeapListener.h"
#include "PointCloud.h"
#include "PCLHand.h"
#include "PCLFinger.h"
#include "SocketClient.h"
#include "Touch.h"
using namespace PQ_SDK_MultiTouch;


enum DataStruct{Hand=1, Finger=2, Head=3, ScreenTouch=4, Rotate=5, Split=6};
enum TouchState{NonValid=0,Down=1,Move=2,Up=3};
enum RotateState{RotateNonValid=0,RotateStart=1,RotateAnticlock=2,RotateClock=3,RotateEnd=4};
enum SplitState{SplitNonValid=0,SplitStart=1,SplitApart=2,SplitClose=3,SplitEnd=4};

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
	bool valid;
	int handId;
	int fingerIds[5];
	int frame;
	Position3D handPosition;
	Orientation handOrientation;
};

//Finger data structure, 41 bytes
struct FingerStruct
{
public:
	DataStruct structType;
	bool valid;
	int fingerId;
	int handId;
	int frame;
	Position3D fingerPosition;
	Orientation fingerOrientation;

};

//Head data structure, 37 bytes
struct HeadStruct
{
public:
	DataStruct structType;
	bool valid;
	int headId;
	int frame;
	Position3D headPosition;
	Orientation headOrientation;
};

//Touch data structure, 25 bytes
//To add touch state
struct TouchStruct
{
public:
	DataStruct structType;
	bool valid;
	int touchId;
	int frame;
	Position2D touchPosition;
	TouchState touchState;
};

//Rotate gesture structure, 29 bytes

struct RotateStruct
{
	DataStruct structType;
	bool valid;
	int frame;
	Position2D rotateFingersPositions[2];
	RotateState rotateState;
};

//Split gesture structure, 29 bytes
struct SplitStruct
{
	DataStruct structType;
	bool valid;
	int frame;
	Position2D splitFingersPosition[2];
	SplitState splitState;
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
const int touchStructSize=25;
const int rotateStructSize=29;
const int splitStructSize=29;

void handConversion(HandStruct handStruct, char * handByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&handStruct.structType);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&handStruct.valid);
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

	memmove(handByte,typeByte,4);
	memmove(&handByte[4],validByte,1);
	memmove(&handByte[5],handIdByte,4);
	memmove(&handByte[9],fingerIdsByte,20);
	memmove(&handByte[29],frameByte,4);
	memmove(&handByte[33],positionXByte,4);
	memmove(&handByte[37],positionYByte,4);
	memmove(&handByte[41],positionZByte,4);
	memmove(&handByte[45],orientationXByte,4);
	memmove(&handByte[49],orientationYByte,4);
	memmove(&handByte[53],orientationZByte,4);
}

void touchConversion(TouchStruct touchStruct, char * touchByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&touchStruct.structType);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&touchStruct.valid);
	//Bytes of touchId
	char* touchIdByte=reinterpret_cast<char *>(&touchStruct.touchId);
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(&touchStruct.frame);
	//Bytes of touch position
	char* positionXByte=reinterpret_cast<char *>(&touchStruct.touchPosition.x);
	char* positionYByte=reinterpret_cast<char *>(&touchStruct.touchPosition.y);
	//Bytes of touch state
	char* stateByte=reinterpret_cast<char *>(&touchStruct.touchState);

	memmove(touchByte,typeByte,4);
	memmove(&touchByte[4],validByte,1);
	memmove(&touchByte[5],touchIdByte,4);
	memmove(&touchByte[9],frameByte,4);
	memmove(&touchByte[13],positionXByte,4);
	memmove(&touchByte[17],positionYByte,4);
	memmove(&touchByte[21],stateByte,4);
}

void fingerConversion(FingerStruct fingerStruct, char * fingerByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&fingerStruct.structType);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&fingerStruct.valid);
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

	memmove(fingerByte,typeByte,4);
	memmove(&fingerByte[4],validByte,1);
	memmove(&fingerByte[5],fingerIdByte,4);
	memmove(&fingerByte[9],handIdByte,4);
	memmove(&fingerByte[13],frameByte,4);
	memmove(&fingerByte[17],positionXByte,4);
	memmove(&fingerByte[21],positionYByte,4);
	memmove(&fingerByte[25],positionZByte,4);
	memmove(&fingerByte[29],orientationXByte,4);
	memmove(&fingerByte[33],orientationYByte,4);
	memmove(&fingerByte[37],orientationZByte,4);
}

void headConversion(HeadStruct headStruct, char * headByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&headStruct.structType);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&headStruct.valid);
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


	memmove(headByte,typeByte,4);
	memmove(&headByte[4],validByte,1);
	memmove(&headByte[5],headIdByte,4);
	memmove(&headByte[9],frameByte,4);
	memmove(&headByte[13],positionXByte,4);
	memmove(&headByte[17],positionYByte,4);
	memmove(&headByte[21],positionZByte,4);
	memmove(&headByte[25],orientationXByte,4);
	memmove(&headByte[29],orientationYByte,4);
	memmove(&headByte[33],orientationZByte,4);
}

void rotateConversion(RotateStruct rotateStruct, char * rotateByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(&rotateStruct.structType);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(&rotateStruct.valid);
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(&rotateStruct.frame);
	//Bytes of fingers position
	char* positionX1Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPositions[0].x);
	char* positionY1Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPositions[0].y);
	char* positionX2Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPositions[1].x);
	char* positionY2Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPositions[1].y);
	//Bytes of rotate state
	char* stateByte=reinterpret_cast<char *>(&rotateStruct.rotateState);
}

void splitConversion(SplitStruct splitStruct, char * SplitByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(& splitStruct.structType);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(& splitStruct.valid);
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(& splitStruct.frame);
	//Bytes of fingers position
	char* positionX1Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPositions[0].x);
	char* positionY1Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPositions[0].y);
	char* positionX2Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPositions[1].x);
	char* positionY2Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPositions[1].y);
	//Bytes of rotate state
	char* stateByte=reinterpret_cast<char *>(&splitStruct.splitState);
}

int main( int argc, char** argv )  
{  
	int err_code=touch.Init();
	if(err_code != PQMTE_SUCCESS){
		cout << "press any key to exit..." << endl;
		getchar();
		return 0;
	}
	// do other things of your application;
	cout << "hello world" << endl;

	//Client socket
	SocketClient client;
	//bool connection=client.ConnectToHost(8000,"192.168.193.200");
	bool connection=client.ConnectToHost(8000,"192.168.1.3");

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

	//Add event listener to the controller.
	controller.addListener(leapListener);

	while(key!=27/*&&connection==true*/)
	{

		//Update kinect for each frame.
		kinectOpenNI.kinectUpdate();
		//Get depth and color image.
		kinectOpenNI.getCVImage(&depthImage,&colorImage);
		//Check if any user is detected.
		userFound=kinectOpenNI.checkUser(&skeletonCap, colorImage);

		frameCount++;

		char headByte[headStructSize];
		char rightHandByte[handStructSize];
		char leftHandByte[handStructSize];
		char handsByte[handStructSize*2];
		char fingersByte[fingerStructSize*10];
		char touchesByte[touchStructSize*10];
		char rotateByte[rotateStructSize];
		char splitByte[splitStructSize];

		HeadStruct headStruct;
		headStruct.structType=DataStruct::Head;
		headStruct.valid=false;
		HandStruct rightHandStruct;
		rightHandStruct.valid=false;
		rightHandStruct.structType=DataStruct::Hand;
		HandStruct leftHandStruct;
		leftHandStruct.valid=false;
		leftHandStruct.structType=DataStruct::Hand;

		RotateStruct rotateStruct;
		rotateStruct.structType=DataStruct::Rotate;
		rotateStruct.valid=false;
		rotateStruct.frame=frameCount;
		rotateStruct.rotateFingersPositions[0].x=-1;
		rotateStruct.rotateFingersPositions[0].y=-1;
		rotateStruct.rotateFingersPositions[1].x=-1;
		rotateStruct.rotateFingersPositions[1].y=-1;
		

		SplitStruct splitStruct;
		splitStruct.structType=DataStruct::Split;
		splitStruct.valid=false;
		splitStruct.frame=frameCount;
		splitStruct.splitFingersPositions[0].x=-1;
		splitStruct.splitFingersPositions[0].y=-1;
		splitStruct.splitFingersPositions[1].x=-1;
		splitStruct.splitFingersPositions[1].y=-1;

		/*HandStruct handStruct[2];
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
		}*/

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

		/*for(int i=0;i<controller.frame().hands().count();++i)
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
		}*/



		/*for(int i=0;i<2;++i)
		{
		char handByte[handStructSize];
		handConversion(handStruct[i],handByte);
		memmove(&handsByte[i*handStructSize],handByte,handStructSize);
		}*/

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
			touchStruct[i].touchState=TouchState::NonValid;
		}

		int resolutionX=touch.getResolutionX();
		int resolutionY=touch.getResolutionY();

		for(int i=0;i<touch.getTouchPointList().size();++i)
		{
			touchStruct[i].valid=true;
			touchStruct[i].touchId=touch.getTouchPointList()[i].id;
			touchStruct[i].touchPosition.x=((float)touch.getTouchPointList()[i].x/(float)resolutionX);
			touchStruct[i].touchPosition.y=((float)touch.getTouchPointList()[i].y/(float)resolutionY);
			switch(touch.getTouchPointList()[i].point_event)
			{
				//Touch Down sometimes can't be obtained correctly, covered by Touch Move
			case TP_DOWN:
				touchStruct[i].touchState=TouchState::Down;
				break;
			case TP_MOVE:
				touchStruct[i].touchState=TouchState::Move;
				break;
			case TP_UP:
				touchStruct[i].touchState=TouchState::Up;
				break;
			}
		}

		int validTouchCount=0;
		for(int i=0;i<10;++i)
		{
			if(touchStruct[i].valid==true)
				validTouchCount++;
			char touchByte[touchStructSize];
			touchConversion(touchStruct[i],touchByte);
			memmove(&touchesByte[i*touchStructSize],touchByte,touchStructSize);
		}

		for(int i=0;i<10;++i)
		{
			if(validTouchCount==1)
			{
				if(touchStruct[i].touchState==TouchState::Up)
				{
					touchStruct[i].valid=false;
					touchStruct[i].touchId=-1;
					touchStruct[i].touchState=TouchState::NonValid;
					touchStruct[i].touchPosition.x=-1;
					touchStruct[i].touchPosition.y=-1;
				}
				touch.getTouchPointList().clear();
			}
		}


		if(userFound==true)
		{
			headStruct.valid=true;
			headStruct.headId=kinectOpenNI.getHeadId();
			headStruct.frame=frameCount;
			headStruct.headPosition.x=kinectOpenNI.getHead().x;
			headStruct.headPosition.y=kinectOpenNI.getHead().y;
			headStruct.headPosition.z=kinectOpenNI.getHead().z;

			rightHandStruct.valid=true;
			leftHandStruct.valid=true;
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
		}
		headConversion(headStruct,headByte);
		handConversion(rightHandStruct,rightHandByte);
		handConversion(leftHandStruct,leftHandByte);

		char dataToSend[headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize];
		memmove(dataToSend,headByte,headStructSize);
		///////////////////Kinect hands
		memmove(&dataToSend[headStructSize],rightHandByte,handStructSize);
		memmove(&dataToSend[headStructSize+handStructSize],leftHandByte,handStructSize);
		///////////////////
		///////////////////Leap hands
		/*memmove(&dataToSend[headStructSize],handsByte,2*handStructSize);*/
		///////////////////
		memmove(&dataToSend[headStructSize+2*handStructSize],touchesByte,touchStructSize*10);
		memmove(&dataToSend[headStructSize+2*handStructSize+10*touchStructSize],fingersByte,fingerStructSize*10);
		send(client.getClientSocket(),dataToSend,headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize,0);


		//Create point cloud for the environment.
		//pointCloud.createCloudXYZ(kinectOpenNI.getDepthData());
		//Display the point cloud.


		/*pclHands.resize(0);
		pclFingers.resize(0);*/


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
		cv::imshow("depth",depthImage);
		cv::imshow("image",colorImage);

		key=cv::waitKey(20);

	}
	//Close the kinect engine.
	kinectOpenNI.KinectClose();
	return 0;  
}  


