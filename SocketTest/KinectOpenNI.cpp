#include "StdAfx.h"
#include "KinectOpenNI.h"

/* 
XN_SKEL_HEAD          = 1,    XN_SKEL_NECK            = 2, 
XN_SKEL_TORSO         = 3,    XN_SKEL_WAIST           = 4, 
XN_SKEL_LEFT_COLLAR        = 5,    XN_SKEL_LEFT_SHOULDER        = 6, 
XN_SKEL_LEFT_ELBOW        = 7,  XN_SKEL_LEFT_WRIST          = 8, 
XN_SKEL_LEFT_HAND          = 9,    XN_SKEL_LEFT_FINGERTIP    =10, 
XN_SKEL_RIGHT_COLLAR    =11,    XN_SKEL_RIGHT_SHOULDER    =12, 
XN_SKEL_RIGHT_ELBOW        =13,  XN_SKEL_RIGHT_WRIST          =14, 
XN_SKEL_RIGHT_HAND      =15,    XN_SKEL_RIGHT_FINGERTIP    =16, 
XN_SKEL_LEFT_HIP          =17,    XN_SKEL_LEFT_KNEE            =18, 
XN_SKEL_LEFT_ANKLE        =19,  XN_SKEL_LEFT_FOOT            =20, 
XN_SKEL_RIGHT_HIP          =21,    XN_SKEL_RIGHT_KNEE          =22, 
XN_SKEL_RIGHT_ANKLE        =23,    XN_SKEL_RIGHT_FOOT          =24     
*/  
//a line will be drawn between start point and corresponding end point  
static int startSkelPoints[14]={1,2,6,6,12,17,6,7,12,13,17,18,21,22};  
static int endSkelPoints[14]={2,3,12,21,17,21,7,9,13,15,18,20,22,24};

// callback function of user generator: new user  
static void XN_CALLBACK_TYPE  NewUser( xn::UserGenerator& generator, XnUserID user,void* pCookie )  
{  
	std::cout << "New user identified: " << user << std::endl;  
	//userGenerator.GetSkeletonCap().LoadCalibrationDataFromFile( user, "UserCalibration.txt" );  
	generator.GetPoseDetectionCap().StartPoseDetection("Psi", user);  
}  

// callback function of user generator: lost user  
static void XN_CALLBACK_TYPE LostUser( xn::UserGenerator& generator, XnUserID user,void* pCookie )  
{  
	std::cout << "User " << user << " lost" << std::endl;  
}  


// callback function of skeleton: calibration start  
static void XN_CALLBACK_TYPE CalibrationStart( xn::SkeletonCapability& skeleton,XnUserID user,void* pCookie )  
{  
	std::cout << "Calibration start for user " <<  user << std::endl;  
}  

// callback function of skeleton: calibration end   
static void XN_CALLBACK_TYPE CalibrationEnd( xn::SkeletonCapability& skeleton,XnUserID user,XnCalibrationStatus calibrationError,void* pCookie )  
{  
	std::cout << "Calibration complete for user " <<  user << ", ";  
	if( calibrationError==XN_CALIBRATION_STATUS_OK )  
	{  
		std::cout << "Success" << std::endl;  
		skeleton.StartTracking( user );  
		//userGenerator.GetSkeletonCap().SaveCalibrationDataToFile(user, "UserCalibration.txt" );  
	}  
	else  
	{  
		std::cout << "Failure" << std::endl;  
		//For the current version of OpenNI, only Psi pose is available  
		((xn::UserGenerator*)pCookie)->GetPoseDetectionCap().StartPoseDetection( "Psi", user );  
	}  
}  

// callback function of pose detection: pose start  
static void XN_CALLBACK_TYPE PoseDetected( xn::PoseDetectionCapability& poseDetection,const XnChar* strPose,XnUserID user,void* pCookie)  
{  
	std::cout << "Pose " << strPose << " detected for user " <<  user << std::endl;  
	((xn::UserGenerator*)pCookie)->GetSkeletonCap().RequestCalibration( user, FALSE );  
	poseDetection.StopPoseDetection( user );  
}  

KinectOpenNI::KinectOpenNI(void)
{
	result = XN_STATUS_OK;    

	// context   
	result = context.Init();   
	CheckOpenNIError( result, "initialize context" );    

	// creategenerator    
	result = depthGenerator.Create( context );   
	CheckOpenNIError( result, "Create depth generator" );    
	result = imageGenerator.Create( context );   
	CheckOpenNIError( result, "Create image generator" );  
	result=userGenerator.Create(context);
	CheckOpenNIError( result, "Create user generator" );  

	//map mode  
	mapMode.nXRes = 640;    
	mapMode.nYRes = 480;   
	mapMode.nFPS = 30;   
	result = depthGenerator.SetMapOutputMode( mapMode );    
	result = imageGenerator.SetMapOutputMode( mapMode );  

	// correct view port    
	depthGenerator.GetAlternativeViewPointCap().SetViewPoint( imageGenerator ); 
}

KinectOpenNI::~KinectOpenNI(void)
{
}

xn::SkeletonCapability KinectOpenNI::KinectRun()
{
	//Handle used to 
	XnCallbackHandle userCBHandle;
	userGenerator.RegisterUserCallbacks(NewUser,LostUser,NULL,userCBHandle);
	userGenerator.GetPoseDetectionCap().RegisterToPoseDetected( PoseDetected,&userGenerator, poseCBHandle );  

	xn::SkeletonCapability skeletonCap=userGenerator.GetSkeletonCap();
	skeletonCap.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	skeletonCap.RegisterToCalibrationStart( CalibrationStart,&userGenerator, calibCBHandle );  
	skeletonCap.RegisterToCalibrationComplete( CalibrationEnd,&userGenerator, calibCBHandle ); 

	result = context.StartGeneratingAll();
	////Unpdate context
	//context.WaitAnyUpdateAll();

	return skeletonCap;
}


const XnDepthPixel*KinectOpenNI::getDepthData()
{
	const XnDepthPixel* pDepth=depthMD.Data();
	return pDepth;

}
const XnUInt8* KinectOpenNI::getImageData()
{
	const XnUInt8* pColor=imageMD.Data();
	return pColor;
}

void KinectOpenNI::kinectUpdate()
{
	context.WaitAndUpdateAll();
}
//void KinectOpenNI::KinectRun()
//{
//	XnCallbackHandle userCBHandle;
//	userGenerator.RegisterUserCallbacks(NewUser,LostUser,NULL,userCBHandle);
//	xn::SkeletonCapability skeletonCap=userGenerator.GetSkeletonCap();
//   
//	skeletonCap.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
//
//	skeletonCap.RegisterToCalibrationStart( CalibrationStart,&userGenerator, calibCBHandle );  
//	skeletonCap.RegisterToCalibrationComplete( CalibrationEnd,&userGenerator, calibCBHandle ); 
// 
//    userGenerator.GetPoseDetectionCap().RegisterToPoseDetected( PoseDetected,&userGenerator, poseCBHandle );  
//
//	//read data  
//    result = context.StartGeneratingAll();   
//
//	pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
//
//	key=0;
//	mode=1;
//
//	
//
//	while(key!=27)
//	{
//		context.WaitAnyUpdateAll();
//
//        //get meta data  
//        depthGenerator.GetMetaData(depthMD);   
//        imageGenerator.GetMetaData(imageMD); 
//		
//		getCVImage(&depthImage,&colorImage);
//		bool getUser=checkUser(&skeletonCap);
//		displayImage();
//		
//	
//	
//		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSample=pointCloud.downSampling(pointCloud.getCloudXYZ(),5.0f,5.0f,5.0f);
//		//pcl::PointCloud<pcl::PointXYZ>::Ptr zCloud=pointCloud.passThroughFilter(cloudDownSample,"z",0.0f,1500.0f);
//		////pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud=pointCloud.getCloudPlane(zCloud);
//
//		//for(int i=0;i<1000;++i)
//		//{
//		//	pointCloud.searchNeighbourKdTreeRadius (zCloud,40,&zCloud->points[i]);
//		//}
//		//cloudViewer.showCloud(zCloud);
//		
//		if(getUser==true)
//		{	
//			
//			//filter the points too far
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPassThrough=pointCloud.passThroughFilter(pointCloud.getCloudXYZ(),"z",0.0f,2000.0f);
//			//downsample the point cloud
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSample=pointCloud.downSampling(cloudPassThrough,5.0f,5.0f,5.0f);
//			//Get the potential hand point cloud
//			pcl::PointCloud<pcl::PointXYZ>::Ptr potentialCloud(new pcl::PointCloud<pcl::PointXYZ>);
//			pcl::PointCloud<pcl::PointXYZ>::Ptr potentialRightHand=pointCloud.searchNeighbourKdTreeRadius(cloudDownSample,200.0f,&rightHand);//200
//
//			pcl::PointCloud<pcl::PointXYZ>::Ptr leftHandCloud(new pcl::PointCloud<pcl::PointXYZ>);
//			pcl::PointCloud<pcl::PointXYZ>::Ptr rightHandCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	        
//			pcl::PointCloud<pcl::PointXYZ> potentialRight=*potentialRightHand;
//			pcl::PointCloud<pcl::PointXYZ> test;
//			
//			
//			if(potentialRightHand->points.size()!=0)
//			{
//				Eigen::Vector4f* leftNearCent;
//				Eigen::Vector4f* rightNearCent;
//				//Get right hand point cloud
//			    if(pointCloud.getNearBlobs2( potentialRightHand,leftHandCloud,rightHandCloud)==false)
//				{
//					pointCloud.setArmCenter(&rightElbow,0);
//				}
//				//Calculate the direction of the hand
//				pointCloud.getEigens(rightHandCloud,0);
//				pcl::PointCloud<pcl::PointXYZ>::Ptr palm(new pcl::PointCloud<pcl::PointXYZ>);
//				pcl::PointCloud<pcl::PointXYZ>::Ptr digits(new pcl::PointCloud<pcl::PointXYZ>); 
//				//Get the cloud of digits
//				//pointCloud.radiusFilter(rightHandCloud,300,20,0,palm,digits);
//				pointCloud.covarianceFilter(rightHandCloud,30,0,palm,digits);
//
//				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>fingers;
//				if(digits->points.size()!=0)
//				{
//					//Segmentation of fingers
//					fingers=pointCloud.segFingers(digits,7,20);//5,20
//				}
//
//			
//				if(key=='0')
//					mode=0;
//				if(key=='1')
//					mode=1;
//				if(key=='2')
//					mode=2;
//				if(key=='3')
//					mode=3;
//				if(key=='4')
//					mode=4;
//				if(key=='5')
//					mode=5;
//				if(key=='6')
//					mode=6;
//				if(key=='7')
//					mode=7;
//
//		
//				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorHand(new pcl::PointCloud<pcl::PointXYZRGBA>);
//				std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>colorFingers;
//				for(int i=0;i<fingers.size();++i)
//				{
//					/*if(pointCloud.checkFinger(fingers[i])==false)
//						continue;*/
//					
//				
//					/*if(pointCloud.checkFinger(fingers[i])<0.86)
//						continue;*/
//					//If the finger tip is not far enough to the center, pass it
//					double distance=pointCloud.checkFingerDistance(fingers[i]);
//					if(distance<70)
//					continue;
//						std::cout<<distance<<" "<<i<<std::endl;
//				    std::cout<<std::endl;
//
//					switch(i)
//					{
//					case 0:
//						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,0,0));
//						break;
//					case 1:
//						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],0,255,0));
//						break;
//					case 2:
//						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],0,0,255));
//						break;
//					case 3:
//						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,255,0));
//						break;
//					case 4:
//						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],0,255,255));
//						break;
//					case 5:
//						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,0,255));
//						break;
//					case 6:
//						colorFingers.push_back(pointCloud.getColorPointCloud(fingers[i],255,255,255));
//						break;
//					}
//				}
//
//			
//				//Add finger cloud to color hand cloud
//				for(int i=0;i<colorFingers.size();++i)
//				{
//					for(int j=0;j<colorFingers[i]->points.size();++j)
//					{
//						colorHand->points.push_back(colorFingers[i]->points[j]);
//					}
//				}
//				//////////Add center and direction line
//				pcl::PointXYZRGBA handCenter;
//				handCenter.x=pointCloud.getHandCenter()(0);
//				handCenter.y=pointCloud.getHandCenter()(1);
//				handCenter.z=pointCloud.getHandCenter()(2);
//				handCenter.r=100;
//				handCenter.g=50;
//				handCenter.b=200;
//				colorHand->points.push_back(handCenter);
//
//				for(int i=0;i<50;++i)
//				{
//					pcl::PointXYZRGBA point;
//					point.x=handCenter.x+i*pointCloud.getHandDirection()(0);
//					point.y=handCenter.y+i*pointCloud.getHandDirection()(1);
//					point.z=handCenter.z+i*pointCloud.getHandDirection()(2);
//					point.r=100;
//					point.g=50;
//					point.b=200;
//					colorHand->points.push_back(point);
//				}
//
//				for(int i=0;i<fingers.size();++i)
//				{
//					double distance=pointCloud.checkFingerDistance(fingers[i]);
//					if(distance<70)
//					continue;
//					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fingerLineCloud=pointCloud.getFingerLine(fingers[i]);
//					for(int j=0;j<fingerLineCloud->points.size();++j)
//					{
//						colorHand->points.push_back(fingerLineCloud->points[j]);
//					}
//				}
//
//				////////////////
//
//				colorHand->width=colorHand->points.size();
//				colorHand->height=1;
//				colorHand->resize(colorHand->width);
//
//				if(mode==0)
//				{
//
//				}
//				if(mode==1)
//				{
//					cloudViewer.showCloud(rightHandCloud);
//				}
//				if(mode==2)
//				{
//					cloudViewer.showCloud(digits);
//				}
//				if(mode==3)
//				{
//					cloudViewer.showCloud(colorHand);
//				}
//				if(mode==4)
//				{
//					cloudViewer.showCloud(potentialRightHand);
//				}
//				if(mode==5)
//				{}
//				if(mode==6)
//				{}
//				if(mode==7)
//				{}
//			}
//		
//		}
//		
//	}
//}

void KinectOpenNI::KinectClose()
{
	//Stop generating the context
	context.StopGeneratingAll();  
	context.Shutdown();  
}

void KinectOpenNI::CheckOpenNIError( XnStatus result, std::string status )  
{   
	if( result != XN_STATUS_OK )   
		std::cerr << status << " Error: " << xnGetStatusString( result ) << std::endl;  
}  

void KinectOpenNI::getCVImage(cv::Mat* depthImage,cv::Mat* colorImage)
{
	//get meta data  
	depthGenerator.GetMetaData(depthMD);   
	imageGenerator.GetMetaData(imageMD); 

	//OpenCV output  
	cv::Mat rawDepthImage(480,640,CV_16UC1,(void*)depthMD.Data());
	rawDepthImage.convertTo( *depthImage, CV_8U, 255.0 / 5000 );

	cv::Mat rawColorImage(480,640,CV_8UC3,(void*)imageMD.Data());
	cv::cvtColor( rawColorImage,*colorImage, CV_RGB2BGR );
}

//void KinectOpenNI::getCVImage(cv::Mat* depthImage,cv::Mat* colorImage)
//{
//	//OpenCV output  
//	cv::Mat rawDepthImage(480,640,CV_16UC1,(void*)depthMD.Data());
//	rawDepthImage.convertTo( *depthImage, CV_8U, 255.0 / 5000 );
//
//	cv::Mat rawColorImage(480,640,CV_8UC3,(void*)imageMD.Data());
//	cv::cvtColor( rawColorImage,*colorImage, CV_RGB2BGR );
//}


bool KinectOpenNI::checkUser(xn::SkeletonCapability* skeletonCap, cv::Mat colorImage)
{
	XnUInt16 userCounts=userGenerator.GetNumberOfUsers();
	bool detected=false;
	if(userCounts>0)
	{
		XnUserID* userID=new XnUserID[userCounts];

		userGenerator.GetUsers(userID,userCounts);


		for(int i=0;i<userCounts;i++)
		{
			if(skeletonCap->IsTracking(userID[i]))
			{
				principalUserId=userID[i];

				XnPoint3D skelPointsIn[24],skelPointsOut[24];  
				XnPoint3D skelPointsInOrien[24],skelPointsOutOrien[24];
				XnSkeletonJointTransformation mJointTran;  
				for(int iter=0;iter<24;iter++)
				{
					skeletonCap->GetSkeletonJoint( userID[i],XnSkeletonJoint(iter+1), mJointTran );  
					skelPointsIn[iter]=mJointTran.position.position;  
					
				}
				depthGenerator.ConvertRealWorldToProjective(24,skelPointsIn,skelPointsOut);
				detected=true;

				for(int d=0;d<14;d++)  
				{  
					CvPoint startpoint = cvPoint(skelPointsOut[startSkelPoints[d]-1].X,skelPointsOut[startSkelPoints[d]-1].Y);  
					CvPoint endpoint = cvPoint(skelPointsOut[endSkelPoints[d]-1].X,skelPointsOut[endSkelPoints[d]-1].Y);  

					cv::circle(colorImage,startpoint,3,CV_RGB(0,0,255),12);
					cv::circle(colorImage,endpoint,3,CV_RGB(0,0,255),12);
					cv::line(colorImage,startpoint,endpoint,CV_RGB(0,0,255),4);
				}  

				float F = 0.0019047619f;
				/////////////////
				//Head ID is 1, skelPointsOut index is 0
				headId=principalUserId*100+1;

				head.x=(skelPointsOut[0].X-320)*skelPointsOut[0].Z*F;
				head.y=(skelPointsOut[0].Y-240)*skelPointsOut[0].Z*F;
				head.z=skelPointsOut[0].Z;

				//RightHand ID is 15, skelPointsOut index is 14
				rightHandId=principalUserId*100+15;

				rightHand.x=(skelPointsOut[14].X-320)*skelPointsOut[14].Z*F;
				rightHand.y=(skelPointsOut[14].Y-240)*skelPointsOut[14].Z*F;
				rightHand.z=skelPointsOut[14].Z;

				//LeftHand ID is 9, skelPointsOut index is 8
				leftHandId=principalUserId*100+9;

				leftHand.x=(skelPointsOut[8].X-320)*skelPointsOut[8].Z*F;
				leftHand.y=(skelPointsOut[8].Y-240)*skelPointsOut[8].Z*F;
				leftHand.z=skelPointsOut[8].Z;

				//RightElbow ID is 13, skelPointsOut index is 12
				rightElbow.x=(skelPointsOut[12].X-320)*skelPointsOut[12].Z*F;
				rightElbow.y=(skelPointsOut[12].Y-240)*skelPointsOut[12].Z*F;
				rightElbow.z=skelPointsOut[12].Z;

				//LeftElbow ID is 7, skelPointsOut index is 6
				leftElbow.x=(skelPointsOut[6].X-320)*skelPointsOut[6].Z*F;
				leftElbow.y=(skelPointsOut[6].Y-240)*skelPointsOut[6].Z*F;
				leftElbow.z=skelPointsOut[6].Z;
				break;
			}
		}
	}

	/*const XnDepthPixel* pDepth=depthMD.Data();
	const XnUInt8* pColor=imageMD.Data();
	pointCloud.createCloudXYZ(pDepth);*/
	/*pointCloud.createCloudXYZRGBA(pDepth,pColor);*/

	if(detected==true)
	{
		//pointCloud.createCloudXYZ(pDepth);
		return true;
	}
	else
	{
		return false;
	}
}

//void KinectOpenNI::displayImage()
//{
//	cvNamedWindow("depth",1);  
//		cvNamedWindow("image",1);
//
//		cv::imshow("depth",depthImage);
//		cv::imshow("image",colorImage);
//
//		key=cv::waitKey(20);
//}

XnUserID KinectOpenNI::getUserID()
{
	return principalUserId;
}

int KinectOpenNI::getHeadId()
{
	return headId;
}
int KinectOpenNI::getRightHandId()
{
	return rightHandId;
}

int KinectOpenNI::getLeftHandId()
{
	return leftHandId;
}

pcl::PointXYZ KinectOpenNI::getHead()
{
	return head;
}

pcl::PointXYZ KinectOpenNI::getRightHand()
{
	return rightHand;
}

pcl::PointXYZ KinectOpenNI::getLeftHand()
{
	return leftHand;
}