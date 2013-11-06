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
const int nonValidValue=9999;

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

//Rotate gesture structure, 33 bytes
struct RotateStruct
{
	DataStruct structType;
	bool valid;
	int frame;
	Position2D rotateFingersPosition[2];
	RotateState rotateState;
	float rotateAngle;
};

//Split gesture structure, 37 bytes
struct SplitStruct
{
	DataStruct structType;
	bool valid;
	int frame;
	Position2D splitFingersPosition[2];
	SplitState splitState;
	float fingerDistance;
	float ratio;
};

//Keyboard input
char key=0;
//Display mode
int mode=0;
//Matrix of depth image
cv::Mat depthImage;
//Matrix of color image
cv::Mat colorImage;
//Matrix of raw depth image
cv::Mat rawDepthImage;
//PointCloud
PointCloud pointCloud;
//Point cloud of left hand
PointCloud leftPointCloud;
//Point cloud of right hand
PointCloud rightPointCloud;
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
const int rotateStructSize=33;
const int splitStructSize=37;

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
	char* positionX1Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPosition[0].x);
	char* positionY1Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPosition[0].y);
	char* positionX2Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPosition[1].x);
	char* positionY2Byte=reinterpret_cast<char *>(&rotateStruct.rotateFingersPosition[1].y);

	//Bytes of rotate angle
	char* angleByte=reinterpret_cast<char *>(&rotateStruct.rotateAngle);
	//Bytes of rotate state
	char* stateByte=reinterpret_cast<char *>(&rotateStruct.rotateState);

	memmove(rotateByte,typeByte,4);
	memmove(&rotateByte[4],validByte,1);
	memmove(&rotateByte[5],frameByte,4);
	memmove(&rotateByte[9],positionX1Byte,4);
	memmove(&rotateByte[13],positionY1Byte,4);
	memmove(&rotateByte[17],positionX2Byte,4);
	memmove(&rotateByte[21],positionY2Byte,4);
	memmove(&rotateByte[25],angleByte,4);
	memmove(&rotateByte[29],stateByte,4);
}

void splitConversion(SplitStruct splitStruct, char * splitByte)
{
	//Bytes of structure type
	char* typeByte=reinterpret_cast<char *>(& splitStruct.structType);
	//Bytes of valid value
	char* validByte=reinterpret_cast<char *>(& splitStruct.valid);
	//Bytes of frame
	char* frameByte=reinterpret_cast<char *>(& splitStruct.frame);
	//Bytes of fingers position
	char* positionX1Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPosition[0].x);
	char* positionY1Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPosition[0].y);
	char* positionX2Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPosition[1].x);
	char* positionY2Byte=reinterpret_cast<char *>(& splitStruct.splitFingersPosition[1].y);
	//Bytes of distance of fingers
	char* distanceByte=reinterpret_cast<char *>(& splitStruct.fingerDistance);
	//Bytes of ratio of distance
	char* ratioByte=reinterpret_cast<char *>(& splitStruct.ratio);
	//Bytes of rotate state
	char* stateByte=reinterpret_cast<char *>(&splitStruct.splitState);

	memmove(splitByte,typeByte,4);
	memmove(&splitByte[4],validByte,1);
	memmove(&splitByte[5],frameByte,4);
	memmove(&splitByte[9],positionX1Byte,4);
	memmove(&splitByte[13],positionY1Byte,4);
	memmove(&splitByte[17],positionX2Byte,4);
	memmove(&splitByte[21],positionY2Byte,4);
	memmove(&splitByte[25],distanceByte,4);
	memmove(&splitByte[29],ratioByte,4);
	memmove(&splitByte[33],stateByte,4);
}

void initialHandStruct(HandStruct &handStruct, int frame)
{
	handStruct.structType=DataStruct::Hand;
	handStruct.valid=false;
	handStruct.handId=nonValidValue;
	for(int i=0;i<5;++i)
	{
		handStruct.fingerIds[i]=nonValidValue;
	}
	handStruct.frame=frame;
	handStruct.handPosition.x=nonValidValue;
	handStruct.handPosition.y=nonValidValue;
	handStruct.handPosition.z=nonValidValue;
	handStruct.handOrientation.x=nonValidValue;
	handStruct.handOrientation.y=nonValidValue;
	handStruct.handOrientation.z=nonValidValue;
}

void initialFingerStruct(FingerStruct &fingerStruct, int frame)
{
	fingerStruct.structType=DataStruct::Finger;
	fingerStruct.valid=false;
	fingerStruct.fingerId=nonValidValue;
	fingerStruct.handId=nonValidValue;
	fingerStruct.frame=frame;
	fingerStruct.fingerPosition.x=nonValidValue;
	fingerStruct.fingerPosition.y=nonValidValue;
	fingerStruct.fingerPosition.z=nonValidValue;
	fingerStruct.fingerOrientation.x=nonValidValue;
	fingerStruct.fingerOrientation.y=nonValidValue;
	fingerStruct.fingerOrientation.z=nonValidValue;
}

void initialHeadStruct(HeadStruct &headStruct, int frame)
{
	headStruct.structType=DataStruct::Head;
	headStruct.valid=false;
	headStruct.headId=nonValidValue;
	headStruct.frame=frame;
	headStruct.headPosition.x=nonValidValue;
	headStruct.headPosition.y=nonValidValue;
	headStruct.headPosition.z=nonValidValue;
	headStruct.headOrientation.x=nonValidValue;
	headStruct.headOrientation.y=nonValidValue;
	headStruct.headOrientation.z=nonValidValue;
}

void initialTouchStruct(TouchStruct &touchStruct,int frame)
{
	touchStruct.structType=DataStruct::ScreenTouch;
	touchStruct.valid=false;
	touchStruct.touchId=nonValidValue;
	touchStruct.frame=frame;
	touchStruct.touchPosition.x=nonValidValue;
	touchStruct.touchPosition.y=nonValidValue;
	touchStruct.touchState=TouchState::NonValid;
}

void initialRotateStruct(RotateStruct &rotateStruct,int frame)
{
	rotateStruct.structType=DataStruct::Rotate;
	rotateStruct.valid=false;
	rotateStruct.frame=frame;
	rotateStruct.rotateFingersPosition[0].x=nonValidValue;
	rotateStruct.rotateFingersPosition[0].y=nonValidValue;
	rotateStruct.rotateFingersPosition[1].x=nonValidValue;
	rotateStruct.rotateFingersPosition[1].y=nonValidValue;
	rotateStruct.rotateState=RotateState::RotateNonValid;
	rotateStruct.rotateAngle=nonValidValue;
}

void initialSplitStruct(SplitStruct &splitStruct,int frame)
{
	splitStruct.structType=DataStruct::Split;
	splitStruct.valid=false;
	splitStruct.frame=frame;
	splitStruct.splitFingersPosition[0].x=nonValidValue;
	splitStruct.splitFingersPosition[0].y=nonValidValue;
	splitStruct.splitFingersPosition[1].x=nonValidValue;
	splitStruct.splitFingersPosition[1].y=nonValidValue;
	splitStruct.splitState=SplitState::SplitNonValid;
	splitStruct.fingerDistance=nonValidValue;
	splitStruct.ratio=nonValidValue;
}

void printLeapInput(Controller controller)
{
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
}

void getFingerInput(FingerStruct& fingerStruct,Leap::Finger finger)
{
	fingerStruct.valid=true;
	fingerStruct.fingerId=finger.id();
	fingerStruct.handId=finger.hand().id();
	fingerStruct.fingerPosition.x=finger.tipPosition().x;
	fingerStruct.fingerPosition.y=finger.tipPosition().y;
	fingerStruct.fingerPosition.z=finger.tipPosition().z;
	fingerStruct.fingerOrientation.x=finger.direction().x;
	fingerStruct.fingerOrientation.y=finger.direction().y;
	fingerStruct.fingerOrientation.z=finger.direction().z;
}

void getHandInput(HandStruct& handStruct,Leap::Hand hand)
{
	handStruct.valid=true;
	handStruct.handId=hand.id();
	for(int i=0;i<hand.fingers().count();++i)
	{
		handStruct.fingerIds[i]=hand.fingers()[i].id();
	}
	handStruct.handPosition.x=hand.palmPosition().x;
	handStruct.handPosition.y=hand.palmPosition().y;
	handStruct.handPosition.z=hand.palmPosition().z;
	handStruct.handOrientation.x=hand.direction().x;
	handStruct.handOrientation.y=hand.direction().y;
	handStruct.handOrientation.z=hand.direction().z;
}

void getHeadInput(HeadStruct& headStruct,KinectOpenNI& kinectOpenNI)
{
	headStruct.valid=true;
	headStruct.headId=kinectOpenNI.getHeadId();
	headStruct.headPosition.x=kinectOpenNI.getHead().x;
	headStruct.headPosition.y=kinectOpenNI.getHead().y;
	headStruct.headPosition.z=kinectOpenNI.getHead().z;
}

void getRightHandInput(HandStruct& rightHandStruct,KinectOpenNI& kinectOpenNI)
{
	rightHandStruct.valid=true;	
	rightHandStruct.handId=kinectOpenNI.getRightHandId();
	rightHandStruct.handPosition.x=kinectOpenNI.getRightHand().x;
	rightHandStruct.handPosition.y=kinectOpenNI.getRightHand().y;
	rightHandStruct.handPosition.z=kinectOpenNI.getRightHand().z;
}

void getLeftHandInput(HandStruct& leftHandStruct,KinectOpenNI& kinectOpenNI)
{
	leftHandStruct.valid=true;
	leftHandStruct.handId=kinectOpenNI.getLeftHandId();
	leftHandStruct.handPosition.x=kinectOpenNI.getLeftHand().x;
	leftHandStruct.handPosition.y=kinectOpenNI.getLeftHand().y;
	leftHandStruct.handPosition.z=kinectOpenNI.getLeftHand().z;
}

void getRotateInput(RotateStruct& rotateStruct, double* rotateParameters,int resolutionX, int resolutionY)
{
	rotateStruct.valid=true;
	rotateStruct.rotateFingersPosition[0].x=((float)rotateParameters[1]/(float)resolutionX);
	rotateStruct.rotateFingersPosition[0].y=((float)rotateParameters[2]/(float)resolutionY);
	rotateStruct.rotateFingersPosition[1].x=((float)rotateParameters[3]/(float)resolutionX);
	rotateStruct.rotateFingersPosition[1].y=((float)rotateParameters[4]/(float)resolutionY);
	rotateStruct.rotateAngle=rotateParameters[5];
}

void getSplitInput(SplitStruct& splitStruct, double* splitParameters,int resolutionX, int resolutionY)
{
	splitStruct.valid=true;
	splitStruct.splitFingersPosition[0].x=((float)splitParameters[1]/(float)resolutionX);
	splitStruct.splitFingersPosition[0].y=((float)splitParameters[2]/(float)resolutionY);
	splitStruct.splitFingersPosition[1].x=((float)splitParameters[3]/(float)resolutionX);
	splitStruct.splitFingersPosition[1].y=((float)splitParameters[4]/(float)resolutionY);
	splitStruct.fingerDistance=splitParameters[5];
	splitStruct.ratio=splitParameters[6];
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
	//bool connection=client.ConnectToHost(8000,"192.168.1.3");

	//Create KinectOpenNI engine.
	KinectOpenNI kinectOpenNI;
	//Run Kinect.
	xn::SkeletonCapability skeletonCap=kinectOpenNI.KinectRun();
	//Create point cloud viewer.
	pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	
	viewer.setBackgroundColor (0, 0, 0);
	//////color
	
	/*viewer.addPointCloud<pcl::PointXYZRGBA> (pointCloud.getCloudXYZRGBA(), rgb, "sample cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");*/


	
  
 
  

	////////////////////

	/*viewer.addPointCloud<pcl::PointXYZ> (pointCloud.getCloudXYZ(), "sample cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");*/
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();
	


	
  
 



	//Point cloud of Leap data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr leapCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudDownSample;

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);*/

	

	//Create depth image window.
	cvNamedWindow("depth",1);  
	//Create color image window.
	cvNamedWindow("image",1);
	//Create derivative image window
	cvNamedWindow("derivative",1);
	//Create hand image window
	cvNamedWindow("hands",1);

	//Add event listener to the controller.
	controller.addListener(leapListener);

	while(key!=27/*&&connection==true*/)
	{

		//Update kinect for each frame.
		kinectOpenNI.kinectUpdate();
		//Get depth and color image.
		rawDepthImage=kinectOpenNI.getCVImage(&depthImage,&colorImage);
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

		//Initial Kinect head
		HeadStruct headStruct;
		initialHeadStruct(headStruct,frameCount);
		//Initial Kinecft hands
		HandStruct rightHandStruct;
		initialHandStruct(rightHandStruct,frameCount);
		HandStruct leftHandStruct;
		initialHandStruct(leftHandStruct,frameCount);
		//Initial rotate gesture
		RotateStruct rotateStruct;
		initialRotateStruct(rotateStruct,frameCount);
		//Initial split gesture
		SplitStruct splitStruct;
		initialSplitStruct(splitStruct,frameCount);

		//Initial Leap hands
		/*HandStruct handStruct[2];
		for(int i=0;i<2;++i)
		{
		initialHandStruct(handStruct[i],frameCount);
		}*/

		//Get Leap hands
		/*for(int i=0;i<controller.frame().hands().count();++i)
		{
			getHandInput(handStruct[i],hand);
		}*/

		//Convert Leap hands to bytes
		/*for(int i=0;i<2;++i)
		{
		char handByte[handStructSize];
		handConversion(handStruct[i],handByte);
		memmove(&handsByte[i*handStructSize],handByte,handStructSize);
		}*/

		//Initial Leap fingers
		FingerStruct fingerStruct[10];
		for(int i=0;i<10;++i)
		{
			initialFingerStruct(fingerStruct[i],frameCount);
		}

		//Get Leap fingers
		for(int i=0;i<controller.frame().fingers().count();++i)
		{
			Leap::Finger finger=controller.frame().fingers()[i];
			getFingerInput(fingerStruct[i],finger);
		}

		//Convert Leap fingers to bytes
		for(int i=0;i<10;++i)
		{
			char fingerByte[fingerStructSize];
			fingerConversion(fingerStruct[i],fingerByte);
			memmove(&fingersByte[i*fingerStructSize],fingerByte,fingerStructSize);
		}

		//Initial touch
		TouchStruct touchStruct[10];
		for(int i=0;i<10;++i)
		{
			initialTouchStruct(touchStruct[i],frameCount);
		}

		int resolutionX=touch.getResolutionX();
		int resolutionY=touch.getResolutionY();

		//Get touch input
		for(int i=0;i<touch.getTouchPointList().size();++i)
		{
			touchStruct[i].valid=true;
			touchStruct[i].touchId=touch.getTouchPointList()[i].id;
			//Get the unified coordinates
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

		

		double *rotateParameters=touch.getRotateParameters();
		//Check if the rotate touches are the same as original touches
		/*double newRotateParameters[6];
		for(int i=0;i<6;++i)
		{
			newRotateParameters[i]=rotateParameters[i];
		}

		if(rotateParameters[0]!=0&&touch.getTouchPointList().size()==2)
		{
			for(int i=0;i<touch.getTouchPointList().size();++i)
			{
				int disX1=std::abs(touch.getTouchPointList()[i].x-rotateParameters[1]);
				int disY1=std::abs(touch.getTouchPointList()[i].y-rotateParameters[2]);
				int disX2=std::abs(touch.getTouchPointList()[i].x-rotateParameters[3]);
				int disY2=std::abs(touch.getTouchPointList()[i].y-rotateParameters[4]);
				int dis1=pow(disX1*disX1+disY1*disY1,0.5);
				int dis2=pow(disX2*disX2+disY2*disY2,0.5);
				if(dis1<dis2)
				{
					newRotateParameters[1]=touch.getTouchPointList()[i].x;
					newRotateParameters[2]=touch.getTouchPointList()[i].y;
				}
				if(dis1>dis2)
				{
					newRotateParameters[3]=touch.getTouchPointList()[i].x;
					newRotateParameters[4]=touch.getTouchPointList()[i].y;
				}
			}
		}*/

		switch((int)rotateParameters[0])
		{
		case 1:
			rotateStruct.rotateState=RotateState::RotateStart;
			getRotateInput(rotateStruct,rotateParameters,resolutionX,resolutionY);
			cout<<"RotateStart"<<endl;
			break;
		case 2:
			rotateStruct.rotateState=RotateState::RotateAnticlock;
			getRotateInput(rotateStruct,rotateParameters,resolutionX,resolutionY);
			cout<<"Anticlock"<<frameCount<<endl;
			break;
		case 3:
			rotateStruct.rotateState=RotateState::RotateClock;
			getRotateInput(rotateStruct,rotateParameters,resolutionX,resolutionY);
			cout<<"Clock"<<frameCount<<endl;
			break;
		case 4:
			rotateStruct.rotateState=RotateState::RotateEnd;
			getRotateInput(rotateStruct,rotateParameters,resolutionX,resolutionY);
			cout<<"RotateEnd"<<endl;
			break;
		default:
			rotateStruct.rotateState=RotateState::RotateNonValid;
			break;
		}
		if(rotateParameters[0]==4)
		{
			rotateParameters[0]=0;
			rotateParameters[1]=nonValidValue;
			rotateParameters[2]=nonValidValue;
			rotateParameters[3]=nonValidValue;
			rotateParameters[4]=nonValidValue;
			rotateParameters[5]=nonValidValue;
		}
		

		rotateConversion(rotateStruct, rotateByte);

		
		

		double *splitParameters=touch.getSplitParameters();

		switch((int)splitParameters[0])
		{
		case 1:
			splitStruct.splitState=SplitState::SplitStart;
			getSplitInput(splitStruct,splitParameters,resolutionX,resolutionY);
			cout<<"SplitStart"<<endl;
			break;
		case 2:
			splitStruct.splitState=SplitState::SplitApart;
			getSplitInput(splitStruct,splitParameters,resolutionX,resolutionY);
			cout<<"SplitApart"<<frameCount<<endl;
			break;
		case 3:
			splitStruct.splitState=SplitState::SplitClose;
			getSplitInput(splitStruct,splitParameters,resolutionX,resolutionY);
			cout<<"SplitClose"<<frameCount<<endl;
			break;
		case 4:
			splitStruct.splitState=SplitState::SplitEnd;
			getSplitInput(splitStruct,splitParameters,resolutionX,resolutionY);
			cout<<"SplitEnd"<<endl;
			break;
		default:
			splitStruct.splitState=SplitState::SplitNonValid;
			break;
		}

		if(splitParameters[0]==4)
		{
			splitParameters[0]=0;
			splitParameters[1]=nonValidValue;
			splitParameters[2]=nonValidValue;
			splitParameters[3]=nonValidValue;
			splitParameters[4]=nonValidValue;
			splitParameters[5]=nonValidValue;
			splitParameters[5]=nonValidValue;
		}

		splitConversion(splitStruct, splitByte);
		//Convert touches to bytes
		int validTouchCount=0;
		for(int i=0;i<10;++i)
		{
			if(touchStruct[i].valid==true)
				validTouchCount++;
			char touchByte[touchStructSize];
			touchConversion(touchStruct[i],touchByte);
			memmove(&touchesByte[i*touchStructSize],touchByte,touchStructSize);
		}

		//If the only existing touch is up, clear the touchlist
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

		//Get Kinect head and hands input
		if(userFound==true)
		{
			getHeadInput(headStruct,kinectOpenNI);
			getRightHandInput(rightHandStruct,kinectOpenNI);
			getLeftHandInput(leftHandStruct,kinectOpenNI);
		}
		//Convert Kinect input to bytes
		headConversion(headStruct,headByte);
		handConversion(rightHandStruct,rightHandByte);
		handConversion(leftHandStruct,leftHandByte);

		char dataToSend[headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize+rotateStructSize+splitStructSize];
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
		memmove(&dataToSend[headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize],rotateByte,rotateStructSize);
		memmove(&dataToSend[headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize+rotateStructSize],splitByte,splitStructSize);
		send(client.getClientSocket(),dataToSend,headStructSize+2*handStructSize+10*touchStructSize+10*fingerStructSize+rotateStructSize+splitStructSize,0);

		//////////////////////Hand detection
		//Create point cloud for the environment.
		//pointCloud.createCloudXYZ(kinectOpenNI.getDepthData());
		//pointCloud.createCloudXYZRGBA(kinectOpenNI.getDepthData(),kinectOpenNI.getImageData());
		/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr zCloud=pointCloud.passThroughFilter(pointCloud.getCloudXYZRGBA(),"z",0.0f,1500.0f);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudDownSample=pointCloud.downSampling(zCloud,5.0f,5.0f,5.0f);*/
		//cloudDownSample=pointCloud.downSampling(zCloud,5.0f,5.0f,5.0f);
		
		////Calculate the normals
		//pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> ne;
		//ne.setInputCloud(cloudDownSample);

		//pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
		//ne.setSearchMethod(tree);

		//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		//ne.setRadiusSearch(30);
		//ne.compute(*cloud_normals);
		
		

		
		

  
  
		
		//viewer.removeAllPointClouds();
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloudDownSample);
		//viewer.addPointCloud<pcl::PointXYZRGBA> (cloudDownSample, rgb, "sample cloud");
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		//viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloudDownSample, cloud_normals, 10, 0.05, "normals");
		////viewer.addPointCloud<pcl::PointXYZRGBA> (pointCloud.getCloudXYZRGBA(), "sample cloud");
		//viewer.spinOnce(10);


		//cloudViewer.showCloud(pointCloud.getCloudXYZ());
		//cloudViewer.showCloud(pointCloud.getCloudXYZRGBA());
		////////////////////////////////
		




		/*pclHands.resize(0);
		pclFingers.resize(0);*/


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



		//printLeapInput(controller);

		//leapCloud->points.resize(0);

		//for(int i=0;i<pclHands.size();++i)
		//{
		//	leapCloud->points.push_back(*pclHands.at(i));
		//}

		//for(int i=0;i<pclFingers.size();++i)
		//{
		//	leapCloud->points.push_back(*pclFingers.at(i));
		//}



		
		//////////Image processing
		cv::Mat grad_x, grad_y;
		cv::Mat abs_grad_x,abs_grad_y;
		cv::Mat derivative;
		cv::Mat forthGroundDepth;
		cv::Mat leftHandMat,rightHandMat;
		cv::Mat hands;
		cv::Mat rawLeftHand,rawRightHand;

		int scale=1;
		int delta=0;
		int ddepth=CV_16S;

		
		depthImage.copyTo(forthGroundDepth);
		rawDepthImage.copyTo(rawLeftHand);
		rawDepthImage.copyTo(rawRightHand);

		for(int i=0;i<forthGroundDepth.size().height;i++)
		{
			for(int j=0;j<forthGroundDepth.size().width;j++)
			{
				if(forthGroundDepth.at<uchar>(i,j)>180)
				{
					forthGroundDepth.at<uchar>(i,j)=0;	
					rawLeftHand.at<unsigned short>(i,j)=0;
					rawRightHand.at<unsigned short>(i,j)=0;
				}			
			}
		}


		forthGroundDepth.copyTo(leftHandMat);
		forthGroundDepth.copyTo(rightHandMat);

		
		if(userFound==true)
		{
			XnPoint3D imageHead=kinectOpenNI.getImageHead();
			XnPoint3D imageRightHand=kinectOpenNI.getImageRightHand();
			XnPoint3D imageLeftHand=kinectOpenNI.getImageLeftHand();
			
			for(int i=0;i<480;i++)
			{
				for(int j=0;j<640;j++)
				{
					bool leftCheck=false;
					bool rightCheck=false;

					if(i<imageRightHand.Y+50&&i>imageRightHand.Y-50&&j<imageRightHand.X+50&&j>imageRightHand.X-50)
						rightCheck=true;
					if(i<imageLeftHand.Y+50&&i>imageLeftHand.Y-50&&j<imageLeftHand.X+50&&j>imageLeftHand.X-50)
						leftCheck=true;

					if(leftCheck==false)
					{
						leftHandMat.at<unsigned char>(i,j)=0;
						rawLeftHand.at<unsigned short>(i,j)=0;
					}
						
					if(rightCheck==false)
					{
						rightHandMat.at<unsigned char>(i,j)=0;
						rawRightHand.at<unsigned short>(i,j)=0;
					}
						
				}
			}

			/*leftPointCloud.createCloudXYZRGBA(&rawLeftHand,kinectOpenNI.getImageData());
			rightPointCloud.createCloudXYZRGBA(&rawRightHand,kinectOpenNI.getImageData());*/

			pointCloud.createCloudXYZ(&rawLeftHand);
			pcl::PointCloud<pcl::PointXYZ>::Ptr leftCloudDownSample=pointCloud.downSampling(pointCloud.getCloudXYZ(),2.0f,2.0f,2.0f);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr potentialLeftHand=pointCloud.searchNeighbourKdTreeRadius(leftCloudDownSample,250.0f,&kinectOpenNI.getLeftHand());
			pcl::PointCloud<pcl::PointXYZ>::Ptr leftHandCloud(new pcl::PointCloud<pcl::PointXYZ>);;
			pcl::PointXYZ leftElbow;
			if(potentialLeftHand->size()!=0)
			{
				if(pointCloud.getNearBlobs2(potentialLeftHand,leftHandCloud)==false)
				{
					pointCloud.setArmCenter(&leftElbow,0);
				}
			}

			pointCloud.getEigens(leftHandCloud,0);

			pcl::PointCloud<pcl::PointXYZ>::Ptr palm(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr digits(new pcl::PointCloud<pcl::PointXYZ>); 

			pointCloud.covarianceFilter(leftHandCloud,35,0,palm,digits);//30
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>fingers;
			if(digits->points.size()!=0)
			{
				//Segmentation of fingers
				fingers=pointCloud.segFingers(digits,7,20);//5,20
			}

			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorHand(new pcl::PointCloud<pcl::PointXYZRGBA>);
			std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>colorFingers;


				for(int i=0;i<fingers.size();++i)
				{
					/*if(pointCloud.checkFinger(fingers[i])==false)
					continue;*/


					/*if(pointCloud.checkFinger(fingers[i])<0.86)
					continue;*/
					//If the finger tip is not far enough to the center, pass it
					double distance=pointCloud.checkFingerDistance(fingers[i]);
					if(distance<70)
						continue;
					std::cout<<distance<<" "<<i<<std::endl;
					std::cout<<std::endl;

					switch(i)
					{
					case 0:
						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,0,0));
						break;
					case 1:
						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],0,255,0));
						break;
					case 2:
						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],0,0,255));
						break;
					case 3:
						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,255,0));
						break;
					case 4:
						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],0,255,255));
						break;
					case 5:
						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,0,255));
						break;
					case 6:
						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,255,255));
						break;
					}
				}


				//Add finger cloud to color hand cloud
				for(int i=0;i<colorFingers.size();++i)
				{
					for(int j=0;j<colorFingers[i]->points.size();++j)
					{
						colorHand->points.push_back(colorFingers[i]->points[j]);
					}
				}
				//////////Add center and direction line
				pcl::PointXYZRGBA handCenter;
				handCenter.x=pointCloud.getHandCenter()(0);
				handCenter.y=pointCloud.getHandCenter()(1);
				handCenter.z=pointCloud.getHandCenter()(2);
				handCenter.r=100;
				handCenter.g=50;
				handCenter.b=200;
				colorHand->points.push_back(handCenter);

				for(int i=0;i<50;++i)
				{
					pcl::PointXYZRGBA point;
					point.x=handCenter.x+i*pointCloud.getHandDirection()(0);
					point.y=handCenter.y+i*pointCloud.getHandDirection()(1);
					point.z=handCenter.z+i*pointCloud.getHandDirection()(2);
					point.r=100;
					point.g=50;
					point.b=200;
					colorHand->points.push_back(point);
				}

				for(int i=0;i<fingers.size();++i)
				{
					double distance=pointCloud.checkFingerDistance(fingers[i]);
					if(distance<70)
						continue;
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fingerLineCloud=pointCloud.getFingerLine(fingers[i]);
					for(int j=0;j<fingerLineCloud->points.size();++j)
					{
						colorHand->points.push_back(fingerLineCloud->points[j]);
					}
				}
			switch(key)
			{
			case '1':
				mode=1;
				break;
			case '2':
				mode=2;
				break;
			case '3':
				mode=3;
				break;
			case '4':
				mode=4;
				break;
			}

			

			switch(mode)
			{
			case 1:
				cloudViewer.showCloud(leftCloudDownSample);
				break;
			case 2:
				cloudViewer.showCloud(leftHandCloud);
				break;
			case 3:
				cloudViewer.showCloud(digits);
				break;
			case 4:
				cloudViewer.showCloud(colorHand);
				break;
				
				
			}
			

		/*	
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leftCloudDownSample=leftPointCloud.downSampling(pointCloud.getCloudXYZRGBA(),5.0f,5.0f,5.0f);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rightCloudDownSample=rightPointCloud.downSampling(pointCloud.getCloudXYZRGBA(),5.0f,5.0f,5.0f);*/

			
			
			
			//Create point cloud for the environment.
		
		////Calculate the normals
			/*pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> ne;
			ne.setInputCloud(cloudDownSample);

			pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
			ne.setSearchMethod(tree);

			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
			ne.setRadiusSearch(5);
			ne.compute(*cloud_normals);*/

			//viewer.removeAllPointClouds();
			//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloudDownSample);
			//viewer.addPointCloud<pcl::PointXYZRGBA> (cloudDownSample, rgb, "sample cloud");
			//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
			//viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloudDownSample, cloud_normals, 10, 0.05, "normals");
			////viewer.addPointCloud<pcl::PointXYZRGBA> (pointCloud.getCloudXYZRGBA(), "sample cloud");
			//viewer.spinOnce(10);
		
		}

		
		

		Sobel(forthGroundDepth,grad_x,ddepth,1,0,3,scale, delta, cv::BORDER_DEFAULT );
		convertScaleAbs(grad_x,abs_grad_x);

		Sobel(forthGroundDepth, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
		convertScaleAbs( grad_y, abs_grad_y );

		addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, derivative );
		addWeighted(leftHandMat,0.5,rightHandMat,0.5,0,hands);


		

		//Display depth and color image.
		cv::imshow("depth",forthGroundDepth);
		cv::imshow("image",colorImage);
		cv::imshow("derivative",derivative);
		cv::imshow("hands",hands);
		///////////////

		
		/*while (!viewer.wasStopped ())
		{
			viewer.spinOnce ();
			cv::imshow("depth",depthImage);
		cv::imshow("image",colorImage);
		cv::imshow("derivative",derivative);
		}*/

		key=cv::waitKey(20);
		
	}
	//Close the kinect engine.
	kinectOpenNI.KinectClose();
	return 0;  
}  


