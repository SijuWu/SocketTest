#include "stdafx.h"
//#include <stdlib.h>
//#include <iostream>
//#include <string>
//#include <opencv\cv.hpp>
//#include <pcl\point_cloud.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//#include "SocketClient.h"
//#include <opencv\highgui.h>
//using namespace std;
//using namespace cv;
//
////Data transfered from the client, the depth image and the color image
////Each pixel is represented by 5 bytes, the first two mean the depth, and the 
////last three mean the RGB
//char buffer[640*480*5];
//
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
//
//int _tmain(int argc, _TCHAR* argv[])
//{
//	//Client socket
//	SocketClient client;
//	//Connect the server
//	//bool connection=client.ConnectToHost(8000,"192.168.1.85");
//	bool connection=client.ConnectToHost(8000,"192.168.33.14");
//	char key=0;
//
//	point_cloud_ptr->width=640;
//	point_cloud_ptr->height=480;
//	point_cloud_ptr->resize(point_cloud_ptr->width*point_cloud_ptr->height);
//
//
//	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//
//
//	while( (key!=27)&&connection==true)
//	{
//
//		memset(buffer,0,sizeof(buffer));
//		int iResult=recv(client.getClientSocket(),buffer,sizeof(buffer),0);
//
//
//		/*std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >*points=&point_cloud_ptr->points;
//		pcl::PointXYZ* point0Ptr=&point_cloud_ptr->points[0];
//		pcl::PointXYZ* point1Ptr=&point_cloud_ptr->points[1];
//		
//		float* x0Ptr=&(*points)[0].x;
//		float* y0Ptr=&(*points)[0].y;
//		float* z0Ptr=&(*points)[0].z;
//		float* x1Ptr=&(*points)[1].x;
//		float* y1Ptr=&(*points)[1].y;
//		float* z1Ptr=&(*points)[1].z;
//		float* ptr1=++x0Ptr;
//		float* ptr14=x0Ptr+4;
//		pcl::PointXYZ* ptr2=++point0Ptr;*/
//
//		////Depth image
//		//Mat depthImage=Mat::zeros(480,640,CV_8UC1);
//		////Color image
//		//Mat colorImage=Mat::zeros(480,640,CV_8UC3);
//
//		int frame=0;
//		if(iResult>0)
//		{
//		/*	int nRows=depthImage.rows;
//			int nCols=depthImage.cols*3;*/
//
//			/*uchar* depthPointer;
//			uchar* colorPointer;*/
//			//Read the data, save them in depthImage and colorImage
//			//We use pointer to save the data quickly
//
//			float F = 0.0019047619f;
//			point_cloud_ptr->resize(point_cloud_ptr->width*point_cloud_ptr->height);
//			std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA> >*points=&point_cloud_ptr->points;
//			pcl::PointXYZRGBA* start=&point_cloud_ptr->points[0];
//			/*float *px=&(*start).data[0];
//			float *p1=&(*start).data[3];
//			uint8_t *b=&(*start).b;
//			uint8_t*g=&(*start).g;
//			uint8_t *r=&(*start).r;
//			uint8_t*a=&(*start).a;
//			
//			uint32_t *rgba=&(*start).rgba;
//			float *rgb=&(*start).rgb;
//
//			pcl::PointXYZRGBA *p2=start+1;*/
//			
//			float* pointPtr=&(*points)[0].x;
//			
//			for(int i=0;i<640*480;++i)
//			{
//				int row=i/640;
//				int column=i%640;
//				
//				
//				uchar pixelDepth=((buffer[i*5+1] << 8) | buffer[i*5]);
//				float depth=pixelDepth;
//			
//			   /* if(depth!=0)
//				{
//					int q=1;
//				}*/
//				/*if(depth!=0)
//				{
//					*pointPtr=column;
//					*(++pointPtr)=row;
//					*(++pointPtr)=depth;
//					pointPtr=pointPtr+2;
//				}*/
//				
//				//float* xp
//				start->x=column;
//				start->y=row;
//				start->z=depth;
//				start->b=buffer[i*5+4];
//				start->g=buffer[i*5+3];
//				start->r=buffer[i*5+2];
//				start++;
//
//			}
//				
//			
//			/*for(int i=0;i<640;i=i+4)
//			{
//				for(int j=0;j<480;j=j+4)
//				{
//					int index=j*640+i;
//					uchar pixelDepth=((buffer[index*5+1] << 8) | buffer[index*5]);
//					
//					
//					
//					point_cloud_ptr->points[index].x=i;
//					point_cloud_ptr->points[index].y=j;
//					point_cloud_ptr->points[index].z=pixelDepth/1000.0;
//					
//				}
//			}*/
//			
//			/*for(int i=0;i<640*480;i=i+4)
//			{
//				int row=i/640;
//				int column=i%640;
//				uchar pixelDepth=((buffer[i*5+1] << 8) | buffer[i*5]);
//				if(pixelDepth==0)
//					continue;
//				point_cloud_ptr->points[i].x=column;
//				point_cloud_ptr->points[i].y=row;
//				point_cloud_ptr->points[i].z=pixelDepth/1000.0;
//				point_cloud_ptr->points[i].r=buffer[i*5+2];
//				point_cloud_ptr->points[i].g=buffer[i*5+3];
//				point_cloud_ptr->points[i].b=buffer[i*5+4];
//			}*/
//			//for(int i=0;i<nRows;++i)
//			//{
//			//	depthPointer=depthImage.ptr<uchar>(i);
//			//	colorPointer=colorImage.ptr<uchar>(i);
//			//	for(int j=0;j<nCols;j=j+3)
//			//	{
//			//		int columns=j/3;
//			//		int index=i*640+columns;
//			//		uchar pixelDepth=((buffer[index*5+1] << 8) | buffer[index*5]);
//			//		/*	depthPointer[columns]=((buffer[index*5+1] << 8) | buffer[index*5]);
//			//		colorPointer[j]=buffer[index*5+4];
//			//		colorPointer[j+1]=buffer[index*5+3];
//			//		colorPointer[j+2]=buffer[index*5+2];*/
//
//			//		/*if(pixelDepth==0)
//			//		continue;*/
//			//		/*	pcl::PointXYZRGB point;
//			//		point.x=(columns-320)*pixelDepth*F;
//			//		point.y=(240-i)*pixelDepth*F;
//			//		point.z=pixelDepth;
//			//		point.r=buffer[index*5+2];
//			//		point.g=buffer[index*5+3];
//			//		point.b=buffer[index*5+4];
//			//		point_cloud_ptr->push_back(point);*/
//			//		/*	point_cloud_ptr->points[i*640+columns].x=(columns-320)*pixelDepth*F;
//			//		point_cloud_ptr->points[i*640+columns].y=(240-i)*pixelDepth*F;*/
//			//		point_cloud_ptr->points[i*640+columns].x=columns;
//			//		point_cloud_ptr->points[i*640+columns].y=i;
//			//		point_cloud_ptr->points[i*640+columns].z=columns+i;
//			//		/*point_cloud_ptr->points[i*640+columns].r=buffer[index*5+2];
//			//		point_cloud_ptr->points[i*640+columns].g=buffer[index*5+3];
//			//		point_cloud_ptr->points[i*640+columns].b=buffer[index*5+4];*/
//			//	}
//			//}
//			if(frame%4==0)
//			viewer.showCloud(point_cloud_ptr);
//			frame++;
//			//send(client.getClientSocket(),buffer,sizeof(buffer),0);
//		}
//
//		//Display depth image and color image
//		/*namedWindow("Depth image",1);
//		imshow("Depth image",depthImage);
//		namedWindow("Color image",1);
//		imshow("Color image",colorImage);*/
//
//		//key=cv::waitKey(20);
//	}
//
//
//	return 0;
//}
//
