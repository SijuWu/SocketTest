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

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::searchNeighbourOctreeVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution, pcl::PointXYZ* searchPoint)
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

	octree.setInputCloud (cloudSource);
	octree.addPointsFromInputCloud ();

	std::vector<int> pointIdxVec;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZ>);

	if (octree.voxelSearch (*searchPoint, pointIdxVec))
	{
		cloud_neighbours->width=pointIdxVec.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(pointIdxVec.size());

		for (size_t i = 0; i < pointIdxVec.size (); ++i)
			cloud_neighbours->push_back(cloudSource->points[pointIdxVec[i]]);
	}

	return cloud_neighbours;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::searchNeighbourOctreeVoxel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution, pcl::PointXYZRGBA* searchPoint)
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> octree (resolution);

	octree.setInputCloud (cloudSource);
	octree.addPointsFromInputCloud ();

	std::vector<int> pointIdxVec;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZRGBA>);

	if (octree.voxelSearch (*searchPoint, pointIdxVec))
	{
		std::vector<int>nonNullIndex;

		for (size_t i = 0; i < pointIdxVec.size (); ++i)
		{
			if(cloudSource->points[pointIdxVec[i]].x==0&&cloudSource->points[pointIdxVec[i]].y==0&&cloudSource->points[pointIdxVec[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxVec[i]);
		}
		
		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}

	}

	return cloud_neighbours;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::searchNeighbourOctreeKNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,int neighbourNum, pcl::PointXYZ* searchPoint)
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

	octree.setInputCloud (cloudSource);
	octree.addPointsFromInputCloud ();

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZ>);

	if (octree.nearestKSearch (*searchPoint, neighbourNum, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxNKNSearch[i]].x==0&&cloudSource->points[pointIdxNKNSearch[i]].y==0&&cloudSource->points[pointIdxNKNSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxNKNSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}

	return cloud_neighbours;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::searchNeighbourOctreeKNeighbour(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution,int neighbourNum, pcl::PointXYZRGBA* searchPoint)
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> octree (resolution);

	octree.setInputCloud (cloudSource);
	octree.addPointsFromInputCloud ();

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZRGBA>);

	if (octree.nearestKSearch (*searchPoint, neighbourNum, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxNKNSearch[i]].x==0&&cloudSource->points[pointIdxNKNSearch[i]].y==0&&cloudSource->points[pointIdxNKNSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxNKNSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}
	return cloud_neighbours;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::searchNeighbourOctreeRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZ* searchPoint)
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

	octree.setInputCloud (cloudSource);
	octree.addPointsFromInputCloud ();

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZ>);

	if (octree.radiusSearch (*searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxRadiusSearch[i]].x==0&&cloudSource->points[pointIdxRadiusSearch[i]].y==0&&cloudSource->points[pointIdxRadiusSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxRadiusSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}

	return cloud_neighbours;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::searchNeighbourOctreeRadius(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZRGBA* searchPoint)
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> octree (resolution);

	octree.setInputCloud (cloudSource);
	octree.addPointsFromInputCloud ();

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZRGBA>);

	if (octree.radiusSearch (*searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxRadiusSearch[i]].x==0&&cloudSource->points[pointIdxRadiusSearch[i]].y==0&&cloudSource->points[pointIdxRadiusSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxRadiusSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}

	return cloud_neighbours;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::searchNeighbourKdTreeKNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,int neighbourNum, pcl::PointXYZ* searchPoint)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloudSource);

	std::vector<int> pointIdxNKNSearch(neighbourNum);
	std::vector<float> pointNKNSquaredDistance(neighbourNum);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZ>);
	if ( kdtree.nearestKSearch (*searchPoint, neighbourNum, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxNKNSearch[i]].x==0&&cloudSource->points[pointIdxNKNSearch[i]].y==0&&cloudSource->points[pointIdxNKNSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxNKNSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}
	return cloud_neighbours;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::searchNeighbourKdTreeKNeighbour(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,int neighbourNum, pcl::PointXYZRGBA* searchPoint)
{
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
	kdtree.setInputCloud(cloudSource);

	std::vector<int> pointIdxNKNSearch(neighbourNum);
	std::vector<float> pointNKNSquaredDistance(neighbourNum);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if ( kdtree.nearestKSearch (*searchPoint, neighbourNum, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxNKNSearch[i]].x==0&&cloudSource->points[pointIdxNKNSearch[i]].y==0&&cloudSource->points[pointIdxNKNSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxNKNSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}

	return cloud_neighbours;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::searchNeighbourKdTreeRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float radius, pcl::PointXYZ* searchPoint)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloudSource);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZ>);
	if ( kdtree.radiusSearch (*searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxRadiusSearch[i]].x==0&&cloudSource->points[pointIdxRadiusSearch[i]].y==0&&cloudSource->points[pointIdxRadiusSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxRadiusSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}
	return cloud_neighbours;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::searchNeighbourKdTreeRadius(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,float radius, pcl::PointXYZRGBA* searchPoint)
{
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
	kdtree.setInputCloud(cloudSource);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_neighbours(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if ( kdtree.radiusSearch (*searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
		std::vector<int>nonNullIndex;
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
		{
			if(cloudSource->points[pointIdxRadiusSearch[i]].x==0&&cloudSource->points[pointIdxRadiusSearch[i]].y==0&&cloudSource->points[pointIdxRadiusSearch[i]].z==0)
				continue;
			nonNullIndex.push_back(pointIdxRadiusSearch[i]);
		}

		cloud_neighbours->width= nonNullIndex.size();
		cloud_neighbours->height=1;
		cloud_neighbours->resize(nonNullIndex.size());

		for(int i=0;i<nonNullIndex.size();i++)
		{
			cloud_neighbours->points[i]=cloudSource->points[nonNullIndex[i]];
		}
	}

	return cloud_neighbours;
}



bool PointCloud::getNearBlobs2(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud,pcl::PointCloud<pcl::PointXYZRGBA> &leftHandCloud,pcl::PointCloud<pcl::PointXYZRGBA> &rightHandCloud/*, std::vector<Eigen::Vector4f> &nearcents*/)
{
	pcl::PointCloud<pcl::PointXYZRGBA> cloudout;
	pcl::PointXYZRGBA pt,pt1,pt2;
	pt.x=pt.y=pt.z=0;
	int pointCount=cloud.points.size();
	std::vector<int> inds1,inds2/*,inds3(pointCount,1)*/;
	std::vector<float> dists;
	Eigen::Vector4f centroid1,centroid2,nearcent1;

	//find cloest pt to camera
	NNN(cloud,&pt,inds1,dists,2000);
	int ind=0;
	double smallestdist;

	for(int i=0;i<dists.size(); ++i){
		if(dists[i]<smallestdist || i==0 ){
			ind=inds1[i];
			smallestdist=dists[i];
		}
	}

	smallestdist=sqrt(smallestdist);
	pt1=cloud.points[ind];

	NNN(cloud,&pt1,inds2,100);

	/**if(inds2.size()<100)
	{
	return false;
	}*/
	pcl::compute3DCentroid(cloud,inds2,centroid1);
	pt2.x=centroid1(0);
	pt2.y=centroid1(1)-20;
	pt2.z=centroid1(2);
	NNN(cloud,&pt2,inds2,100);

	std::vector<int> temp;
	NNN(cloud,&pt2,temp,150);
	/**if(!(findNearbyPts(cloud,temp,nearcent1)))
	return false;*/

	pcl::compute3DCentroid(cloud,inds2,centroid1);
	pt2.x=centroid1(0);
	pt2.y=centroid1(1)-100;
	pt2.z=centroid1(2);
	NNN(cloud,&pt2,inds2,100);

	getSubCloud(cloud,inds2,cloudout);


	rightHandCloud=cloudout;
	//*if(!foundarm)*/
	//nearcents.push_back(nearcent1);
	return true;
}

bool PointCloud::getNearBlobs2( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr leftHandCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr rightHandCloud/*, std::vector<Eigen::Vector4f> &nearcents*/)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ pt,pt1,pt2;
	pt.x=pt.y=pt.z=0;
	int pointCount=cloud->points.size();
	std::vector<int> inds1,inds2/*,inds3(pointCount,1)*/;
	std::vector<float> dists;
	Eigen::Vector4f centroid1,centroid2,nearcent1;

	//find cloest pt to camera
	NNN(cloud,&pt,inds1,dists,2000);
	int ind=0;
	double smallestdist;

	for(int i=0;i<dists.size(); ++i){
		if(dists[i]<smallestdist || i==0 ){
			ind=inds1[i];
			smallestdist=dists[i];
		}
	}

	smallestdist=sqrt(smallestdist);
	pt1=cloud->points[ind];

	std::cout<<ind;
	NNN(cloud,&pt1,inds2,100);

	/**if(inds2.size()<100)
	{
	return false;
	}*/
	pcl::PointCloud<pcl::PointXYZ> handCloud=*cloud;
	pcl::compute3DCentroid(handCloud,inds2,centroid1);
	pt2.x=centroid1(0);
	pt2.y=centroid1(1)-20;
	pt2.z=centroid1(2);
	NNN(cloud,&pt2,inds2,100);

	std::vector<int> temp;
	NNN(cloud,&pt2,temp,150);
	/**if(!(findNearbyPts(cloud,temp,nearcent1)))
	return false;*/

	pcl::compute3DCentroid(handCloud,inds2,centroid1);
	pt2.x=centroid1(0);
	pt2.y=centroid1(1)-10;
	pt2.z=centroid1(2);
	NNN(cloud,&pt2,inds2,100);

	getSubCloud(cloud,inds2,cloudout);


	//rightHandCloud=cloudout;
	rightHandCloud->points.swap(cloudout->points);
	//*if(!foundarm)*/
	//nearcents.push_back(nearcent1);
	return true;
}
bool PointCloud::getNearBlobs2( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leftHandCloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rightHandCloud/*, std::vector<Eigen::Vector4f> &nearcents*/)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointXYZRGBA pt,pt1,pt2;
	pt.x=pt.y=pt.z=0;
	int pointCount=cloud->points.size();
	std::vector<int> inds1,inds2/*,inds3(pointCount,1)*/;
	std::vector<float> dists;
	Eigen::Vector4f centroid1,centroid2,nearcent1;

	//find cloest pt to camera
	NNN(cloud,&pt,inds1,dists,2000);
	int ind=0;
	double smallestdist;

	for(int i=0;i<dists.size(); ++i){
		if(dists[i]<smallestdist || i==0 ){
			ind=inds1[i];
			smallestdist=dists[i];
		}
	}

	smallestdist=sqrt(smallestdist);
	pt1=cloud->points[ind];

	std::cout<<ind;
	NNN(cloud,&pt1,inds2,100);

	/**if(inds2.size()<100)
	{
	return false;
	}*/
	pcl::PointCloud<pcl::PointXYZRGBA> handCloud=*cloud;
	pcl::compute3DCentroid(handCloud,inds2,centroid1);
	pt2.x=centroid1(0);
	pt2.y=centroid1(1)-20;
	pt2.z=centroid1(2);
	NNN(cloud,&pt2,inds2,100);

	std::vector<int> temp;
	NNN(cloud,&pt2,temp,150);
	/**if(!(findNearbyPts(cloud,temp,nearcent1)))
	return false;*/

	pcl::compute3DCentroid(handCloud,inds2,centroid1);
	pt2.x=centroid1(0);
	pt2.y=centroid1(1)-100;
	pt2.z=centroid1(2);
	NNN(cloud,&pt2,inds2,100);

	getSubCloud(cloud,inds2,cloudout);


	//rightHandCloud=cloudout;
	rightHandCloud->points.swap(cloudout->points);
	//*if(!foundarm)*/
	//nearcents.push_back(nearcent1);
	return true;
}
//bool PointCloud::findNearbyPts(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid)
//{
//	std::vector<int> inds(cloud.size(),1);
//	std::vector<int>nearpts;
//	std::vector<int>temp;
//	for(int i=0;i<cloudpts.size();++i)
//		inds[cloudpts[i]]=-1;
//	for(int i=0;i<cloudpts.size();++i)
//	{
//		if(inds[cloudpts[i]]==-1)
//		{
//			NNN(cloud,&(cloud.points[cloudpts[i]]),temp,50);
//			//NNN(cloud,&cloud.points[cloudpts[i]],temp,50);
//			for(int j=0;i<temp.size();++j)
//			{
//				if(inds[temp[j]]==1)
//				{
//					nearpts.push_back(temp[j]);
//					inds[temp[j]]=2;
//				}
//				else
//					inds[temp[j]]=-2;
//			}
//		}
//	}
//	if(nearpts.size())
//		pcl::compute3DCentroid(cloud,nearpts,centroid);
//	else
//		return false;
//	return true;
//}

void PointCloud::getSubCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZRGBA> &subCloud)
{
	for(int i=0;i<subCloudIndex.size();i++)
	{
		subCloud.points.push_back(cloudSource.points[subCloudIndex[i]]);
	}
}

void PointCloud::getSubCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud)
{
	subCloud->width=cloudSource->width;
	subCloud->height=cloudSource->height;
	subCloud->resize(subCloud->width);

	for(int i=0;i<subCloudIndex.size();i++)
	{
		subCloud->points.push_back(cloudSource->points[subCloudIndex[i]]);
	}
}
void PointCloud::getSubCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr subCloud)
{
	subCloud->width=cloudSource->width;
	subCloud->height=cloudSource->height;
	subCloud->resize(subCloud->width);

	for(int i=0;i<subCloudIndex.size();i++)
	{
		subCloud->points.push_back(cloudSource->points[subCloudIndex[i]]);
	}
}
//performs radius search in a simplistic fashion

void PointCloud::NNN(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, double radius){
	inds.clear();
	double r2=radius*radius;
	double smallerrad=radius/std::sqrt(3.0);
	double diffx,diffy,diffz;
	for(int i=0;i<cloud.points.size(); i++){
		if(cloud.points[i].x==0&&cloud.points[i].y==0&&cloud.points[i].z==0)
			continue;
		//find the distance between the cloud point and the reference in the three dimensions:
		diffx=fabs(cloud.points[i].x - center->x);
		diffy=fabs(cloud.points[i].y - center->y);
		diffz=fabs(cloud.points[i].z - center->z);
		//first find whether the point is in an axis aligned cube around the point -   this is a very quick check
		if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
			//if the point is also in a cube whose circumradius is the search radius, the point is close enough
			if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
				inds.push_back(i);
			//if the point actually falls in between the two cubes, we do the more computationally intensive multiply...
			else//between the boxes: check for actual distance
				if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
					inds.push_back(i);
		}//endif in outer box
	}//endfor all points in cloud
}


void PointCloud::NNN(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, std::vector<float> &dists, double radius){
	inds.clear();
	double r2=radius*radius;
	double diffx,diffy,diffz;
	double sqrdist;
	for(int i=0;i<cloud.points.size(); i++){
		if(cloud.points[i].x==0&&cloud.points[i].y==0&&cloud.points[i].z==0)
			continue;
		//find the distance between the cloud point and the reference in the three dimensions:
		diffx=fabs(cloud.points[i].x - center->x);
		diffy=fabs(cloud.points[i].y - center->y);
		diffz=fabs(cloud.points[i].z - center->z);
		//first find whether the point is in an axis aligned cube around the point -   this is a very quick check
		if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
			//calculate actual distance to the point
			sqrdist=diffx*diffx+diffy*diffy+diffz*diffz;
			if(sqrdist < r2){
				inds.push_back(i);
				dists.push_back(sqrdist);
			}
		}//endif in outer box
	}//endfor all points in cloud
}

void PointCloud::NNN(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, double radius)
{
	inds.clear();
	double r2=radius*radius;
	double smallerrad=radius/std::sqrt(3.0);
	double diffx,diffy,diffz;
	for(int i=0;i<cloud->points.size(); i++){
		if(cloud->points[i].x==0&&cloud->points[i].y==0&&cloud->points[i].z==0)
			continue;
		//find the distance between the cloud point and the reference in the three dimensions:
		diffx=fabs(cloud->points[i].x - center->x);
		diffy=fabs(cloud->points[i].y - center->y);
		diffz=fabs(cloud->points[i].z - center->z);
		//first find whether the point is in an axis aligned cube around the point -   this is a very quick check
		if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
			//if the point is also in a cube whose circumradius is the search radius, the point is close enough
			if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
				inds.push_back(i);
			//if the point actually falls in between the two cubes, we do the more computationally intensive multiply...
			else//between the boxes: check for actual distance
				if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
					inds.push_back(i);
		}//endif in outer box
	}//endfor all points in cloud
}
void PointCloud::NNN(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA* center, std::vector<int> &inds, std::vector<float> &dists, double radius)
{
	inds.clear();
	double r2=radius*radius;
	double diffx,diffy,diffz;
	double sqrdist;
	for(int i=0;i<cloud->points.size(); i++){
		if(cloud->points[i].x==0&&cloud->points[i].y==0&&cloud->points[i].z==0)
			continue;
		//find the distance between the cloud point and the reference in the three dimensions:
		diffx=fabs(cloud->points[i].x - center->x);
		diffy=fabs(cloud->points[i].y - center->y);
		diffz=fabs(cloud->points[i].z - center->z);
		//first find whether the point is in an axis aligned cube around the point -   this is a very quick check
		if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
			//calculate actual distance to the point
			sqrdist=diffx*diffx+diffy*diffy+diffz*diffz;
			if(sqrdist < r2){
				inds.push_back(i);
				dists.push_back(sqrdist);
			}
		}//endif in outer box
	}//endfor all points in cloud
}

void PointCloud::NNN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* center, std::vector<int> &inds, double radius)
{
	inds.clear();
	double r2=radius*radius;
	double smallerrad=radius/std::sqrt(3.0);
	double diffx,diffy,diffz;
	for(int i=0;i<cloud->points.size(); i++){
		if(cloud->points[i].x==0&&cloud->points[i].y==0&&cloud->points[i].z==0)
			continue;
		//find the distance between the cloud point and the reference in the three dimensions:
		diffx=fabs(cloud->points[i].x - center->x);
		diffy=fabs(cloud->points[i].y - center->y);
		diffz=fabs(cloud->points[i].z - center->z);
		//first find whether the point is in an axis aligned cube around the point -   this is a very quick check
		if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
			//if the point is also in a cube whose circumradius is the search radius, the point is close enough
			if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
				inds.push_back(i);
			//if the point actually falls in between the two cubes, we do the more computationally intensive multiply...
			else//between the boxes: check for actual distance
				if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
					inds.push_back(i);
		}//endif in outer box
	}//endfor all points in cloud
}

void PointCloud::NNN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* center, std::vector<int> &inds, std::vector<float> &dists, double radius)
{
	inds.clear();
	double r2=radius*radius;
	double diffx,diffy,diffz;
	double sqrdist;
	for(int i=0;i<cloud->points.size(); i++){
		if(cloud->points[i].x==0&&cloud->points[i].y==0&&cloud->points[i].z==0)
			continue;
		//find the distance between the cloud point and the reference in the three dimensions:
		diffx=fabs(cloud->points[i].x - center->x);
		diffy=fabs(cloud->points[i].y - center->y);
		diffz=fabs(cloud->points[i].z - center->z);
		//first find whether the point is in an axis aligned cube around the point -   this is a very quick check
		if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
			//calculate actual distance to the point
			sqrdist=diffx*diffx+diffy*diffy+diffz*diffz;
			if(sqrdist < r2){
				inds.push_back(i);
				dists.push_back(sqrdist);
			}
		}//endif in outer box
	}//endfor all points in cloud
}