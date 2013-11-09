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

double tolerance=35;
int octreeResolution=5;
float octreeRadius=275;
int palmRadius=45;
int nnThresh=4;
int radius=15;
double clustertol=7;
int mincluster=60;
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

void getFingerInput(FingerStruct& fingerStruct, pcl::PointXYZ* finger, int handId,Eigen::Vector3f fingerOrientation)
{
	fingerStruct.valid=true;
	fingerStruct.handId=handId;
	fingerStruct.fingerPosition.x=finger->x;
	fingerStruct.fingerPosition.y=finger->y;
	fingerStruct.fingerPosition.z=finger->z;
	fingerStruct.fingerOrientation.x=fingerOrientation(0);
	fingerStruct.fingerOrientation.y=fingerOrientation(1);
	fingerStruct.fingerOrientation.z=fingerOrientation(2);
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
	bool connection=client.ConnectToHost(8000,"192.168.1.2");

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

	while(key!=27&&connection==true)
	{

		//Update kinect for each frame.
		kinectOpenNI.kinectUpdate();
		//Get depth and color image.
		rawDepthImage=kinectOpenNI.getCVImage(&depthImage,&colorImage);
		//Check if any user is detected.
		userFound=kinectOpenNI.checkUser(&skeletonCap, colorImage);

		frameCount++;

	
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

			pointCloud.getEigens(leftHandCloud,PointCloud::LeftHand);

			pcl::PointCloud<pcl::PointXYZ>::Ptr palm(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr digits(new pcl::PointCloud<pcl::PointXYZ>); 
			pcl::PointCloud<pcl::PointXYZ>::Ptr digitsForthPart(new pcl::PointCloud<pcl::PointXYZ>); 
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr normalHand(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr densityHand(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr indexFinger(new pcl::PointCloud<pcl::PointXYZ>);
			/////////////////covariance filter
			//if(key=='a')
			//{
			//	tolerance++;
			//	std::cout<<tolerance<<std::endl;
			//}
			//if(key=='z')
			//{
			//	tolerance--;
			//	std::cout<<tolerance<<std::endl;
			//}

			//if(key=='q')
			//{
			//	octreeResolution++;
			//	std::cout<<octreeResolution<<std::endl;
			//}
			//if(key=='s')
			//{
			//	octreeResolution--;
			//	std::cout<<octreeResolution<<std::endl;
			//}

			//if(key=='w')
			//{
			//	octreeRadius++;
			//	std::cout<<octreeRadius<<std::endl;
			//}
			//if(key=='x')
			//{
			//	octreeRadius--;
			//	std::cout<<octreeRadius<<std::endl;
			//}
			//////////////////////////



			if(key=='a')
			{
				octreeResolution+=1;
				std::cout<<octreeResolution<<std::endl;
			}
			if(key=='z')
			{
				octreeResolution-=1;
				std::cout<<octreeResolution<<std::endl;
			}

			if(key=='q')
			{
				palmRadius++;
				std::cout<<palmRadius<<std::endl;
			}
			if(key=='s')
			{
				palmRadius--;
				std::cout<<palmRadius<<std::endl;
			}

			if(key=='w')
			{
				radius++;
				std::cout<<radius<<std::endl;
			}
			if(key=='x')
			{
				radius--;
				std::cout<<radius<<std::endl;
			}

			if(key=='c')
			{
				clustertol++;
				std::cout<<clustertol<<std::endl;
			}
			if(key=='v')
			{
				clustertol--;
				std::cout<<clustertol<<std::endl;
			}

			if(key=='b')
			{
				mincluster++;
				std::cout<<mincluster<<std::endl;
			}
			if(key=='n')
			{
				mincluster--;
				std::cout<<mincluster<<std::endl;
			}

			

			pcl::PointXYZ center;
			//Find potential digits cloud
			pointCloud.radiusFilter(leftHandCloud,octreeResolution,palmRadius,0,digits,&center);

			pcl::PointXYZRGBA handCenter;
			handCenter.x=pointCloud.getHandCenter()(0);
			handCenter.y=pointCloud.getHandCenter()(1);
			handCenter.z=pointCloud.getHandCenter()(2);
			handCenter.r=100;
			handCenter.g=50;
			handCenter.b=200;
			densityHand->points.push_back(handCenter);

			Eigen::Vector3f newDirection;
			newDirection(0)=pointCloud.getHandDirection()(0);
			newDirection(1)=pointCloud.getHandDirection()(1);
			newDirection(2)=pointCloud.getHandDirection()(2);

			//Check hand direction
			Eigen::Vector3f newCenter;
			newCenter(0)=handCenter.x;
			newCenter(1)=handCenter.y;
			newCenter(2)=handCenter.z;

			Eigen::Vector3f dir1=newCenter+newDirection;
			Eigen::Vector3f dir2=newCenter-newDirection;

			if(dir2.dot(dir2)<dir1.dot(dir1))
				newDirection=-newDirection;

			//Remove back part of digits cloud
			for(int i=0;i<digits->size();++i)
			{
				Eigen::Vector3f pointDirection;
				pointDirection(0)=digits->at(i).x-(center.x-30*newDirection(0));
				pointDirection(1)=digits->at(i).y-(center.y-30*newDirection(1));
				pointDirection(2)=digits->at(i).z-(center.z-30*newDirection(2));
	
				if(newDirection.dot(pointDirection)<0)
					continue;

				/*int density=pointCloud.searchNeighbourOctreeRadius(digits,30,radius,&digits->at(i))->size();
			
				if(density<30)
					continue;*/

				digitsForthPart->push_back(digits->at(i));
			}


			//for(int i=0;i<digits->size();++i)
			//{
			//	int density=pointCloud.searchNeighbourOctreeRadius(digits,30,radius,&digits->at(i))->size();
			//	pcl::PointXYZRGBA point;
			//	point.x=digits->at(i).x;
			//	point.y=digits->at(i).y;
			//	point.z=digits->at(i).z;

			//	point.r=0;
			//	point.g=0;
			//	point.b=0;
			//	point.a=0;

			//	Eigen::Vector3f pointDirection;
			//	pointDirection(0)=point.x-center.x;
			//	pointDirection(1)=point.y-center.y;
			//	pointDirection(2)=point.z-center.z;
			//
			//	
			//	
			//	if(newDirection.dot(pointDirection)<0)
			//		continue;

			//	if(density<20)
			//	{
			//		point.r=255;
			//		continue;
			//	}
			//		
			//	if(density>=20&&density<40)
			//	{
			//		point.g=255;
			//		//continue;
			//	}
			//		
			//	if(density>=40&&density<60)
			//		point.b=255;
			//	if(density>=60&&density<80)
			//	{
			//		point.r=255;
			//		point.g=255;
			//	}
			//	if(density>=80&&density<100)
			//	{
			//		point.r=255;
			//		point.b=255;
			//	}
			//	if(density>=100&&density<120)
			//	{
			//		point.g=255;
			//		point.b=255;
			//	}

			//	densityHand->push_back(point);
			//}


			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>fingers;
			if(digitsForthPart->points.size()!=0)
			{
				//Segmentation of fingers
				fingers=pointCloud.segFingers(digitsForthPart,clustertol,mincluster);//5,20
			}

			
			std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>colorFingers;

			std::vector<Eigen::Vector3f> fingersDirection;
			fingersDirection.resize(fingers.size());

			double maxDistance=0;
			int indexFingerIndex=-1;
			
				for(int i=0;i<fingers.size();++i)
				{
					//If the finger tip is not far enough to the center, pass it
					double distance=pointCloud.checkFingerDistance(fingers[i],&fingersDirection[i]);
					if(distance>maxDistance)
					{
						maxDistance=distance;
						indexFingerIndex=i;
						
					}
						
					std::cout<<"Finger"<<i<<":"<<fingers[i]->size()<<std::endl;
					/*std::cout<<"Finger"<<i<<":"<<distance;
					if(i==fingers.size()-1)
						std::cout<<std::endl;*/
					if(newDirection.dot(fingersDirection[i])<0.55)
						continue;

					/*if(distance<70)
					{
						int a=1;
						continue;
					}
						*/
				

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

				

				if(indexFingerIndex!=-1)
				{
					Eigen::Vector3f indexDirection;
					pcl::PointXYZ camera;
					camera.x=0;
					camera.y=0;
					camera.z=0;
					pcl::PointCloud<pcl::PointXYZ>::Ptr neighbour=pointCloud.searchNeighbourKdTreeKNeighbour(fingers[indexFingerIndex],1,&camera);
					pcl::PointCloud<pcl::PointXYZ>::Ptr indexTip=pointCloud.searchNeighbourKdTreeKNeighbour(fingers[indexFingerIndex],200,&neighbour->at(0));
					pointCloud.checkFingerDistance(indexTip,&indexDirection);

					//Draw hand direction
					for(int i=0;i<50;++i)
					{
						pcl::PointXYZ point;
						point.x=neighbour->at(0).x+i*indexDirection(0);
						point.y=neighbour->at(0).y+i*indexDirection(1);
						point.z=neighbour->at(0).z+i*indexDirection(2);

						indexFinger->points.push_back(point);
					}

					if(maxDistance>70)
					{
						for(int j=0;j<indexTip->size();++j)
						{
							indexFinger->push_back(indexTip->at(j));
						}
					}
				}
				
				//Add finger cloud to color hand cloud
				for(int i=0;i<colorFingers.size();++i)
				{
					for(int j=0;j<colorFingers[i]->points.size();++j)
					{
						densityHand->points.push_back(colorFingers[i]->points[j]);
					}
				}


			//Draw hand direction
			for(int i=0;i<50;++i)
			{
				pcl::PointXYZRGBA point;
				point.x=handCenter.x+i*newDirection(0);
				point.y=handCenter.y+i*newDirection(1);
				point.z=handCenter.z+i*newDirection(2);
				point.r=100;
				point.g=50;
				point.b=200;
				densityHand->points.push_back(point);
			}

			
			////////////////normal image
			/*pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
			ne.setInputCloud(leftHandCloud);

			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
			ne.setSearchMethod(tree);

			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
			ne.setRadiusSearch(normalRadius);
			ne.compute(*cloud_normals);

			Eigen::Vector4f handDirection=pointCloud.getHandDirection();
			handDirection(3)=0;
			for(int i=0;i<leftHandCloud->size();++i)
			{
				Eigen::Vector4f pointNormal;
				pointNormal(0)=cloud_normals->at(i).normal_x;
				pointNormal(1)=cloud_normals->at(i).normal_y;
				pointNormal(2)=cloud_normals->at(i).normal_z;
				pointNormal(3)=0;
				
				double dotValue=handDirection.dot(pointNormal);
				
				pcl::PointXYZRGBA point;
				if(dotValue>0.7)
				{
					point.x=leftHandCloud->at(i).x;
					point.y=leftHandCloud->at(i).y;
					point.z=leftHandCloud->at(i).z;
					point.r=255;
					point.g=0;
					point.b=0;
					point.a=0;
				}

				if(dotValue>0.4&&dotValue<=0.7)
				{
					point.x=leftHandCloud->at(i).x;
					point.y=leftHandCloud->at(i).y;
					point.z=leftHandCloud->at(i).z;
					point.r=0;
					point.g=255;
					point.b=0;
					point.a=0;
				}

				if(dotValue<=0.4&&dotValue>0.2)
				{
					point.x=leftHandCloud->at(i).x;
					point.y=leftHandCloud->at(i).y;
					point.z=leftHandCloud->at(i).z;
					point.r=0;
					point.g=0;
					point.b=255;
					point.a=0;
				}

				if(dotValue<=0.2)
					{
					point.x=leftHandCloud->at(i).x;
					point.y=leftHandCloud->at(i).y;
					point.z=leftHandCloud->at(i).z;
					point.r=255;
					point.g=0;
					point.b=255;
					point.a=0;
				}

				normalHand->push_back(point);
			}*/
			///////////////////////////




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
			case '5':
				mode=5;
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
				cloudViewer.showCloud(digitsForthPart);
				break;
			case 4:
				//cloudViewer.showCloud(colorHand);
				cloudViewer.showCloud(densityHand);
				break;
			case 5:
				//cloudViewer.showCloud(colorHand);
				cloudViewer.showCloud(indexFinger);
				break;
				
			}
			

		
		
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



		


		key=cv::waitKey(20);
		
	}
	//Close the kinect engine.
	kinectOpenNI.KinectClose();
	return 0;  
}  


