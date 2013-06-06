#include "stdafx.h"
#include<pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <stdlib.h>  
#include <iostream>  
#include <string>  
//
#include <XnCppWrapper.h>  
#include "opencv/cv.h"  
#include "opencv/highgui.h"  
#include "KinectOpenNI.h"

using namespace std;  
using namespace cv;  

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
int startSkelPoints[14]={1,2,6,6,12,17,6,7,12,13,17,18,21,22};  
int endSkelPoints[14]={2,3,12,21,17,21,7,9,13,15,18,20,22,24};

void CheckOpenNIError( XnStatus result, string status )  
{   
    if( result != XN_STATUS_OK )   
        cerr << status << " Error: " << xnGetStatusString( result ) << endl;  
}  
  
// callback function of user generator: new user  
void XN_CALLBACK_TYPE NewUser( xn::UserGenerator& generator, XnUserID user,void* pCookie )  
{  
    cout << "New user identified: " << user << endl;  
    //userGenerator.GetSkeletonCap().LoadCalibrationDataFromFile( user, "UserCalibration.txt" );  
    generator.GetPoseDetectionCap().StartPoseDetection("Psi", user);  
}  
  
// callback function of user generator: lost user  
void XN_CALLBACK_TYPE LostUser( xn::UserGenerator& generator, XnUserID user,void* pCookie )  
{  
    cout << "User " << user << " lost" << endl;  
}  

// callback function of skeleton: calibration start  
void XN_CALLBACK_TYPE CalibrationStart( xn::SkeletonCapability& skeleton,XnUserID user,void* pCookie )  
{  
    cout << "Calibration start for user " <<  user << endl;  
}  
  
// callback function of skeleton: calibration end   
void XN_CALLBACK_TYPE CalibrationEnd( xn::SkeletonCapability& skeleton,XnUserID user,XnCalibrationStatus calibrationError,void* pCookie )  
{  
    cout << "Calibration complete for user " <<  user << ", ";  
    if( calibrationError==XN_CALIBRATION_STATUS_OK )  
    {  
        cout << "Success" << endl;  
        skeleton.StartTracking( user );  
        //userGenerator.GetSkeletonCap().SaveCalibrationDataToFile(user, "UserCalibration.txt" );  
    }  
    else  
    {  
        cout << "Failure" << endl;  
        //For the current version of OpenNI, only Psi pose is available  
        ((xn::UserGenerator*)pCookie)->GetPoseDetectionCap().StartPoseDetection( "Psi", user );  
    }  
}  

// callback function of pose detection: pose start  
void XN_CALLBACK_TYPE PoseDetected( xn::PoseDetectionCapability& poseDetection,const XnChar* strPose,XnUserID user,void* pCookie)  
{  
    cout << "Pose " << strPose << " detected for user " <<  user << endl;  
    ((xn::UserGenerator*)pCookie)->GetSkeletonCap().RequestCalibration( user, FALSE );  
    poseDetection.StopPoseDetection( user );  
}  

int main( int argc, char** argv )  
{  
 //   XnStatus result = XN_STATUS_OK;    
 //   xn::DepthMetaData depthMD;  
 //   xn::ImageMetaData imageMD;  
 // 
 //   //OpenCV  
 //   IplImage*  imgDepth16u=cvCreateImage(cvSize(640,480),IPL_DEPTH_16U,1);  
 //   IplImage* imgRGB8u=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);  
 //   IplImage*  depthShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);  
 //   IplImage* imageShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);  
 //   cvNamedWindow("depth",1);  
 //   cvNamedWindow("image",1);  
 //   char key=0;  
 // 
 //   //
 //   // context   
 //   xn::Context context;   
 //   result = context.Init();   
 //   CheckOpenNIError( result, "initialize context" );    
 // 
 //   // creategenerator    
 //   xn::DepthGenerator depthGenerator;    
 //   result = depthGenerator.Create( context );   
 //   CheckOpenNIError( result, "Create depth generator" );    
 //   xn::ImageGenerator imageGenerator;  
 //   result = imageGenerator.Create( context );   
 //   CheckOpenNIError( result, "Create image generator" );  
	//xn::UserGenerator userGenerator;
	//result=userGenerator.Create(context);
 //   CheckOpenNIError( result, "Create user generator" );    
 //  
 //   // 
 //   //map mode    
 //   XnMapOutputMode mapMode;   
 //   mapMode.nXRes = 640;    
 //   mapMode.nYRes = 480;   
 //   mapMode.nFPS = 30;   
 //   result = depthGenerator.SetMapOutputMode( mapMode );    
 //   result = imageGenerator.SetMapOutputMode( mapMode );    
 //   
	//XnCallbackHandle userCBHandle;
	//userGenerator.RegisterUserCallbacks(NewUser,LostUser,NULL,userCBHandle);
 //   
	//xn::SkeletonCapability skeletonCap=userGenerator.GetSkeletonCap();
 //  
	//skeletonCap.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	//XnCallbackHandle calibCBHandle;
	//skeletonCap.RegisterToCalibrationStart( CalibrationStart,&userGenerator, calibCBHandle );  
	//skeletonCap.RegisterToCalibrationComplete( CalibrationEnd,&userGenerator, calibCBHandle ); 

	// XnCallbackHandle poseCBHandle;  
 //   userGenerator.GetPoseDetectionCap().RegisterToPoseDetected( PoseDetected,&userGenerator, poseCBHandle );  
	//
	//// correct view port    
 //   depthGenerator.GetAlternativeViewPointCap().SetViewPoint( imageGenerator );   
 // 
 //   //
 //   //read data  
 //   result = context.StartGeneratingAll();    
 //   
 // 
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

	// 


 //   while( (key!=27)  )   
 //   {    
	//	//context.WaitAndUpdateAll();
	//	context.WaitAnyUpdateAll();
 //       //get meta data  
 //       depthGenerator.GetMetaData(depthMD);   
 //       imageGenerator.GetMetaData(imageMD);  
 // 
 //       //  
 //       //OpenCV output  
	//	Mat depthImage(480,640,CV_16UC1,(void*)depthMD.Data());
	//	Mat mScaledDepth;
 //       depthImage.convertTo( mScaledDepth, CV_8U, 255.0 / 5000 );

	//	Mat colorImage(480,640,CV_8UC3,(void*)imageMD.Data());
	//	Mat newColorImage;
	//	cv::cvtColor( colorImage,newColorImage, CV_RGB2BGR );
	//	
	//	XnUInt16 userCounts=userGenerator.GetNumberOfUsers();

	//	XnPoint3D skelPointsOut[24];  
	//	bool detected=false;
	//	if(userCounts>0)
	//	{
	//		XnUserID* userID=new XnUserID[userCounts];
	//		userGenerator.GetUsers(userID,userCounts);
	//		for(int i=0;i<userCounts;i++)
	//		{
	//			if(skeletonCap.IsTracking(userID[i]))
	//			{
	//				  //XnPoint3D skelPointsIn[24],skelPointsOut[24];  
	//				XnPoint3D skelPointsIn[24];
	//				  XnSkeletonJointTransformation mJointTran;  
	//				  for(int iter=0;iter<24;iter++)
	//				  {
	//					  skeletonCap.GetSkeletonJoint( userID[i],XnSkeletonJoint(iter+1), mJointTran );  
	//					  skelPointsIn[iter]=mJointTran.position.position;  
	//				  }
	//				   depthGenerator.ConvertRealWorldToProjective(24,skelPointsIn,skelPointsOut);
	//				   detected=true;
	//				   

	//				   for(int d=0;d<14;d++)  
	//				   {  
	//					   CvPoint startpoint = cvPoint(skelPointsOut[startSkelPoints[d]-1].X,skelPointsOut[startSkelPoints[d]-1].Y);  
	//					   CvPoint endpoint = cvPoint(skelPointsOut[endSkelPoints[d]-1].X,skelPointsOut[endSkelPoints[d]-1].Y);  

	//					   circle(newColorImage,startpoint,3,CV_RGB(0,0,255),12);
	//					   circle(newColorImage,endpoint,3,CV_RGB(0,0,255),12);
	//				       line( newColorImage,startpoint,endpoint,CV_RGB(0,0,255),4);
	//
	//				   }  
	//			}
	//		}
	//	}
	//	imshow("depth",mScaledDepth);
	//	imshow("image",newColorImage);

	//	

	//	if(detected==true)
	//	{
	//		//PCL
	//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//		cloud->width=640;
	//		cloud->height=480;
	//		cloud->resize(640*480);

	//		std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA> >*points=&cloud->points;

	//		float* pointPtr=&(*points)[0].x;


	//		int count=0;


	//		

	//		const XnDepthPixel* pDepth=depthMD.Data();


	//		

	//		const XnUInt8* pColor=imageMD.Data();
	//		const XnUInt8* pRed=pColor;
	//		const XnUInt8* pGreen=++pColor;
	//		const XnUInt8* pBlue=++pColor;

	//		float F = 0.0019047619f;

	//		for( unsigned int i =0; i < 640*480; ++i)  
	//		{
	//			//float zValue=((*pDepthTwo << 8) | *pDepthOne);
	//			float zValue=*pDepth;
	//			int row=i/640;
	//			int col=i%640;

	//			if(zValue!=0)
	//			{

	//				*pointPtr=(col-320)*zValue*F;
	//				pointPtr++;
	//				*pointPtr=(row-240)*zValue*F;
	//				pointPtr++;

	//				(*pointPtr)=zValue;

	//				pointPtr=pointPtr+2;

	//				/*if(detected==true)
	//				{*/
	//				float headX=skelPointsOut[0].X;
	//				float headY=skelPointsOut[0].Y;
	//				if(col==headX&&row==headY)
	//				{
	//					cloud->points[i].r=0;
	//					cloud->points[i].g=0;
	//					cloud->points[i].b=255;
	//				}
	//				/*}*/

	//				uint8_t* colorPtr=(uint8_t*)pointPtr;
	//				*colorPtr=*pBlue;
	//				*colorPtr++;
	//				*colorPtr=*pGreen;
	//				*colorPtr++;
	//				*colorPtr=*pRed;
	//				*colorPtr++;
	//				*colorPtr=0;
	//				colorPtr=colorPtr+13;

	//				pointPtr=(float*)colorPtr;
	//			}

	//			else
	//			{
	//				pointPtr=pointPtr+8;
	//			}

	//			++pDepth;

	//			pBlue=pBlue+3;
	//			pGreen=pGreen+3;
	//			pRed=pRed+3;

	//		}
	//		viewer.showCloud(cloud);
	//	}








	//	key=cvWaitKey(20);  
	//}  
 // 
 //   //destroy  
 //   cvDestroyWindow("depth");  
 //   cvDestroyWindow("image");  
 //   cvReleaseImage(&imgDepth16u);  
 //   cvReleaseImage(&imgRGB8u);  
 //   cvReleaseImage(&depthShow);  
 //   cvReleaseImage(&imageShow);  
 //   context.StopGeneratingAll();  
 //   context.Shutdown();  

KinectOpenNI kinectOpenNI;
kinectOpenNI.KinectRun();
kinectOpenNI.KinectClose();
    return 0;  
}  


