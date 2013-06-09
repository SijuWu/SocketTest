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

	cvNamedWindow("depth",1);  
	cvNamedWindow("image",1);  

	

}

KinectOpenNI::~KinectOpenNI(void)
{
}

void KinectOpenNI::KinectRun()
{
	XnCallbackHandle userCBHandle;
	userGenerator.RegisterUserCallbacks(NewUser,LostUser,NULL,userCBHandle);
	xn::SkeletonCapability skeletonCap=userGenerator.GetSkeletonCap();
   
	skeletonCap.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	skeletonCap.RegisterToCalibrationStart( CalibrationStart,&userGenerator, calibCBHandle );  
	skeletonCap.RegisterToCalibrationComplete( CalibrationEnd,&userGenerator, calibCBHandle ); 
 
    userGenerator.GetPoseDetectionCap().RegisterToPoseDetected( PoseDetected,&userGenerator, poseCBHandle );  

	//read data  
    result = context.StartGeneratingAll();   

	pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");

	key=0;
	mode=1;

	

	while(key!=27)
	{
		context.WaitAnyUpdateAll();

        //get meta data  
        depthGenerator.GetMetaData(depthMD);   
        imageGenerator.GetMetaData(imageMD); 
		
		getCVImage(&depthImage,&colorImage);
		bool getUser=checkUser(&skeletonCap);
		displayImage();
		
	
	

			
		
		if(getUser==true)
		{	
			/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPassThrough=pointCloud.passThroughFilter(pointCloud.getCloudXYZRGBA(),"z",0.0f,2000.0f);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudDownSample=pointCloud.downSampling(cloudPassThrough,5.0f,5.0f,5.0f);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr potentialCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);*/

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPassThrough=pointCloud.passThroughFilter(pointCloud.getCloudXYZ(),"z",0.0f,2000.0f);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSample=pointCloud.downSampling(cloudPassThrough,5.0f,5.0f,5.0f);
			pcl::PointCloud<pcl::PointXYZ>::Ptr potentialCloud(new pcl::PointCloud<pcl::PointXYZ>);

			//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr potentialHead=pointCloud.searchNeighbourKdTreeRadius(cloudDownSample,100.0f,&head);
			//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr potentialLeftHand=pointCloud.searchNeighbourKdTreeRadius(cloudDownSample,100.0f,&leftHand);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr potentialRightHand=pointCloud.searchNeighbourKdTreeRadius(cloudDownSample,200.0f,&rightHand);

			int count=0;
			for(int i=0;i<potentialRightHand->points.size();i++)
			{
				if(potentialRightHand->points[i].x==0&&potentialRightHand->points[i].y==0&&potentialRightHand->points[i].z==0)
					count++;
			}
			std::cout<<count;

		/*	potentialCloud->width=potentialHead->width+potentialLeftHand->width+potentialRightHand->width;
			potentialCloud->height=1;
			potentialCloud->resize(potentialCloud->width*potentialCloud->height);
			potentialCloud->points.insert(potentialCloud->points.end(),potentialHead->points.begin(),potentialHead->points.end());
			potentialCloud->points.insert(potentialCloud->points.end(),potentialLeftHand->points.begin(),potentialLeftHand->points.end());
			potentialCloud->points.insert(potentialCloud->points.end(),potentialRightHand->points.begin(),potentialRightHand->points.end());
			cloudViewer.showCloud(potentialCloud);*/

			 //std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > initialclouds;
			//std::vector<Eigen::Vector4f> arm_center;
			/*pcl::PointCloud<pcl::PointXYZRGBA> leftHandCloud;
			pcl::PointCloud<pcl::PointXYZRGBA> rightHandCloud;*/
			pcl::PointCloud<pcl::PointXYZ>::Ptr leftHandCloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr rightHandCloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ> potentialRight=*potentialRightHand;
			pcl::PointCloud<pcl::PointXYZ> test;
			
			
			if(potentialRightHand->points.size()!=0)
			{
				pointCloud.getNearBlobs2( potentialRightHand,leftHandCloud,rightHandCloud);
				/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr right(&rightHandCloud);*/

				cloudViewer.showCloud(rightHandCloud);
			}
		
		}
		/*else
		cloudViewer.showCloud(cloudDownSample);*/
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud=pointCloud.getCloudPlane(cloudDownSample);
		/*std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters=pointCloud.euclideanClusterExtract(cloudDownSample);
		switch(key)
		{
		case '0':
		mode=0;
		break;
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
		case '6':
		mode=6;
		break;
		default:
		break;
		}
		switch(mode)
		{
		case 0:
		cloudViewer.showCloud(cloudDownSample);
		break;
		case 1:
		if(clusters.size()>=1)
		cloudViewer.showCloud(clusters[0]);
		break;
		case 2:
		if(clusters.size()>=2)
		cloudViewer.showCloud(clusters[1]);
		break;
		case 3:
		if(clusters.size()>=3)
		cloudViewer.showCloud(clusters[2]);
		break;
		case 4:	
		if(clusters.size()>=4)
		cloudViewer.showCloud(clusters[3]);
		break;
		case 5:
		if(clusters.size()>=5)
		cloudViewer.showCloud(clusters[4]);
		break;
		case 6:
		if(clusters.size()>=6)
		cloudViewer.showCloud(clusters[5]);
		break;
		default:
		break;
		}*/









	}
}

void KinectOpenNI::KinectClose()
{
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
	//OpenCV output  
	cv::Mat rawDepthImage(480,640,CV_16UC1,(void*)depthMD.Data());
	rawDepthImage.convertTo( *depthImage, CV_8U, 255.0 / 5000 );

	cv::Mat rawColorImage(480,640,CV_8UC3,(void*)imageMD.Data());
	cv::cvtColor( rawColorImage,*colorImage, CV_RGB2BGR );
}

void KinectOpenNI::displayImage()
{
	cv::imshow("depth",depthImage);
	cv::imshow("image",colorImage);
	key=cv::waitKey(20);
}
bool KinectOpenNI::checkUser(xn::SkeletonCapability* skeletonCap)
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
				XnPoint3D skelPointsIn[24],skelPointsOut[24];  
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
			    
				head.x=(skelPointsOut[0].X-320)*skelPointsOut[0].Z*F;
				head.y=(skelPointsOut[0].Y-240)*skelPointsOut[0].Z*F;
				head.z=skelPointsOut[0].Z;

				rightHand.x=(skelPointsOut[15].X-320)*skelPointsOut[15].Z*F;
				rightHand.y=(skelPointsOut[15].Y-240)*skelPointsOut[15].Z*F;
				rightHand.z=skelPointsOut[15].Z;
			}
		}
	}
	
	const XnDepthPixel* pDepth=depthMD.Data();
	const XnUInt8* pColor=imageMD.Data();
	//pointCloud.createCloudXYZ(pDepth);
	/*pointCloud.createCloudXYZRGBA(pDepth,pColor);*/

	if(detected==true)
	{
		pointCloud.createCloudXYZ(pDepth);
		return true;
	}
	else
	{
	return false;
	}
}