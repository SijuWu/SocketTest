#include "StdAfx.h"
#include "PointCloud.h"


PointCloud::PointCloud(void)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBAPtr (new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloudXYZ=cloudXYZPtr;
	cloudXYZ->width=640;
	cloudXYZ->height=480;
	cloudXYZ->resize(640*480);
	cloudXYZRGBA=cloudXYZRGBAPtr;
	cloudXYZRGBA->width=640;
	cloudXYZRGBA->height=480;
	cloudXYZRGBA->resize(640*480);
}


PointCloud::~PointCloud(void)
{
}

void PointCloud::createCloudXYZ( const XnDepthPixel* pDepth)
{
	cloudXYZ->clear();
	cloudXYZ->resize(640*480);
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >*points=&cloudXYZ->points;

	//Get the pointer of the x coordinate of the first point
	float* pointPtr=&(*points)[0].x;
	//Get the pointer of the depthImage
	
  
	//The focal length of the RGB camera
	float F = 0.0019047619f;

	for( unsigned int i =0; i < 640*480; ++i)  
	{
       //Get the depth of the pixel
		float zValue=*pDepth;

		//Get the 2D position of each pixel
		int row=i/640;
		int col=i%640;

		if(zValue!=0)
		{
			//Set the x coordinate of the point
			*pointPtr=(col-320)*zValue*F;
			//Jump to the pointer of the y coordonate
			pointPtr++;
			//Set the y coordinate of the point
			*pointPtr=(row-240)*zValue*F;
			//Jump to the pointer of the z coordinate
			pointPtr++;
			//Set the z coordinate of the point
			(*pointPtr)=zValue;
			//Jump to the pointer of the blue color
			pointPtr=pointPtr+2;
		}

		else
		{
			//Jump to the next point
			pointPtr=pointPtr+4;
		}

		//Jump to the pointer of the next pixel on the depthImage
		++pDepth;
	}
}

void PointCloud::createCloudXYZRGBA( const XnDepthPixel* pDepth,const XnUInt8* pColor)
{
	cloudXYZRGBA->clear();
	cloudXYZRGBA->resize(640*480);
	std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA> >*points=&cloudXYZRGBA->points;

	//Get the pointer of the x coordinate of the first point
	float* pointPtr=&(*points)[0].x;
	const XnUInt8* colorPtr=pColor;
	const XnUInt8* pRed=colorPtr;
	const XnUInt8* pGreen=++colorPtr;
	const XnUInt8* pBlue=++colorPtr;
	//The focal length of the RGB camera
	float F = 0.0019047619f;

	for( unsigned int i =0; i < 640*480; ++i)  
	{
       //Get the depth of the pixel
		float zValue=*pDepth;

		//Get the 2D position of each pixel
		int row=i/640;
		int col=i%640;

		if(zValue!=0)
		{
			//Set the x coordinate of the point
			*pointPtr=(col-320)*zValue*F;
			//Jump to the pointer of the y coordonate
			pointPtr++;
			//Set the y coordinate of the point
			*pointPtr=(row-240)*zValue*F;
			//Jump to the pointer of the z coordinate
			pointPtr++;
			//Set the z coordinate of the point
			(*pointPtr)=zValue;
			//Jump to the pointer of the blue color
			pointPtr=pointPtr+2;

			//Get the pointer of the blue color
			uint8_t* colorPtr=(uint8_t*)pointPtr;
			//Set the blue color
			*colorPtr=*pBlue;
			//Jump to the green color
			*colorPtr++;
			//Set the green color
			*colorPtr=*pGreen;
			//Jump to the red color
			*colorPtr++;
			//Set the red color
			*colorPtr=*pRed;
			//Jump to the transparence 
			*colorPtr++;
			//Set the transparence
			*colorPtr=0;
			//Jump to the next point
			colorPtr=colorPtr+13;
			//Get the pointer of the next point
			pointPtr=(float*)colorPtr;



		}

		else
		{
			//Jump to the next point
			pointPtr=pointPtr+8;
		}

		//Jump to the pointer of the next pixel on the depthImage
		++pDepth;
		//Jump to the color pointers of the next pixel on the colorImage
		pBlue=pBlue+3;
		pGreen=pGreen+3;
		pRed=pRed+3;
	}
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::getCloudXYZ()
{
	return cloudXYZ;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::getCloudXYZRGBA()
{
	return cloudXYZRGBA;
}


//template<typename PointT> pcl::PointCloud<PointT>::Ptr PointCloud::downSampling(pcl::PointCloud<PointT>::Ptr cloudSource,float xLeafSize,float yLeafSize, float zLeafSize)
//{
//	pcl::VoxelGrid<PointT> filtering;
//	filtering.setInputCloud(cloudSource);
//	filtering.setLeafSize(xLeafSize,yLeafSize,zLeafSize);
//	pcl::PointCloud<PointT>::Ptr cloudXYZ_filtered (new pcl::PointCloud<PointT>);
//	filtering.filter(*cloud_filtered);
//	return cloud_filtered;
//}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,float xLeafSize,float yLeafSize, float zLeafSize)
{
	pcl::VoxelGrid<pcl::PointXYZ> filtering;
	filtering.setInputCloud(cloudXYZ);
	filtering.setLeafSize(xLeafSize,yLeafSize,zLeafSize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	filtering.filter(*cloudXYZ_filtered);
	return cloudXYZ_filtered;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::downSampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,float xLeafSize,float yLeafSize, float zLeafSize)
{
	pcl::VoxelGrid<pcl::PointXYZRGBA> filtering;
	filtering.setInputCloud(cloudXYZRGBA);
	filtering.setLeafSize(xLeafSize,yLeafSize,zLeafSize);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	filtering.filter(*cloudXYZRGBA_filtered);
	return cloudXYZRGBA_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ,std::string axis,float minLimit, float maxLimit)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloudXYZ);
	 pass.setFilterFieldName (axis);
	pass.setFilterLimits(minLimit,maxLimit);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pass.filter(*cloudXYZ_filtered);
	return cloudXYZ_filtered;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA,std::string axis,float minLimit, float maxLimit)
{
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloudXYZRGBA);
	 pass.setFilterFieldName (axis);
	pass.setFilterLimits(minLimit,maxLimit);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pass.filter(*cloudXYZRGBA_filtered);
	return cloudXYZRGBA_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::getCloudPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);

	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(10);

	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i=0,nr_points=(int) cloudSource->points.size();
	while(cloudSource->points.size()>0.7*nr_points)
	{
		seg.setInputCloud(cloudSource);
		seg.segment(*inliers,*coefficients);
		if(inliers->indices.size()==0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		extract.setInputCloud(cloudSource);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloudSource.swap(cloud_f);

		i++;
	}
	return cloud_p;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::getCloudPlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);

	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(10);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	int i=0,nr_points=(int) cloudSource->points.size();
	while(cloudSource->points.size()>0.7*nr_points)
	{
		seg.setInputCloud(cloudSource);
		seg.segment(*inliers,*coefficients);
		if(inliers->indices.size()==0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		extract.setInputCloud(cloudSource);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		
		extract.setNegative(true);
		extract.filter(*cloud_f);
		//cloudSource.swap(cloud_f);
		*cloudSource=*cloud_f;
		i++;
	}
	return cloud_p;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloud::euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource)
{
	getCloudPlane(cloudSource);
	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloudSource);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(30);
	ec.setMinClusterSize(500);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudSource);
	ec.extract(cluster_indices);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>cloud_clusters;
	int j=0;
	for(std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
			cloud_cluster->points.push_back(cloudSource->points[*pit]);
		cloud_cluster->width=cloud_cluster->points.size();
		cloud_cluster->height=1;
		cloud_cluster->is_dense=true;
		cloud_clusters.push_back(cloud_cluster);
		j++;
	}
	
	return cloud_clusters;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> PointCloud::euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource)
{
	getCloudPlane(cloudSource);
	 pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloudSource);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(30);
	ec.setMinClusterSize(500);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudSource);
	ec.extract(cluster_indices);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>cloud_clusters;
	int j=0;
	for(std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
			cloud_cluster->points.push_back(cloudSource->points[*pit]);
		cloud_cluster->width=cloud_cluster->points.size();
		cloud_cluster->height=1;
		cloud_cluster->is_dense=true;
		cloud_clusters.push_back(cloud_cluster);
		j++;
	}
	
	return cloud_clusters;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::searchNeighbourOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,pcl::PointXYZ* searchPoint)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	octree.setInputCloud(cloudSource);
	octree.addPointsFromInputCloud();

	std::vector<int> pointIdxVec;

	
	if(octree.voxelSearch(*searchPoint,pointIdxVec))
	{
		cloud_neighbours->width=pointIdxVec.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(pointIdxVec.size());

		for(size_t i=0;i<pointIdxVec.size();++i)
		{
			cloud_neighbours->push_back(cloudSource->points[pointIdxVec[i]]);
		}
	}

	return cloud_neighbours;
	
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::searchNeighbourOctree(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution,pcl::PointXYZRGBA* searchPoint)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> octree(resolution);

	octree.setInputCloud(cloudSource);
	octree.addPointsFromInputCloud();

	std::vector<int> pointIdxVec;
	
	if(octree.voxelSearch(*searchPoint,pointIdxVec))
	{
		cloud_neighbours->width=pointIdxVec.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(pointIdxVec.size());

		for(size_t i=0;i<pointIdxVec.size();++i)
		{
			cloud_neighbours->push_back(cloudSource->points[pointIdxVec[i]]);
		}
	}

	return cloud_neighbours;
}
//pcl::RangeImage PointCloud::getRangeImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ)
//{
//	float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
//	float maxAngleWidth     = (float) (57.0f * (M_PI/180.0f));  // 360.0 degree in radians
//	float maxAngleHeight    = (float) (43.0f * (M_PI/180.0f));  // 180.0 degree in radians
//
//	pcl::PointCloud<pcl::PointXYZ>& point_cloud = *cloudXYZ;
//	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
//	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//	float noiseLevel=0.00;
//	float minRange = 0.0f;
//	int borderSize = 1;
//
//	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
//	pcl::RangeImage& range_image = *range_image_ptr;   
//
//	
//	range_image.createFromPointCloud(point_cloud, angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//
//	return range_image;
//}

//pcl::RangeImage PointCloud::getRangeImage(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA)
//{
//	pcl::PointCloud<pcl::PointXYZRGBA>& point_cloud = *cloudXYZRGBA;
//	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
//	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//	float noiseLevel=0.00;
//	float minRange = 0.0f;
//	int borderSize = 1;
//
//	pcl::RangeImage rangeImage;
//	rangeImage.createFromPointCloud(point_cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//
//	return rangeImage;
//}