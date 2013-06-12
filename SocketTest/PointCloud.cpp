#include "StdAfx.h"
#include "PointCloud.h"

static pcl::PointXYZ eigenToPclPoint(const Eigen::Vector4f &v){
   pcl::PointXYZ p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
}

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

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloud::euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,double tolerance,int minClusterSize,int maxClusterSize)
{
	getCloudPlane(cloudSource);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloudSource);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	/*ec.setClusterTolerance(30);
	ec.setMinClusterSize(500);
	ec.setMaxClusterSize(25000);*/
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

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> PointCloud::euclideanClusterExtract(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource,double tolerance,int minClusterSize,int maxClusterSize)
{
	getCloudPlane(cloudSource);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloudSource);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	//ec.setClusterTolerance(30);
	//ec.setMinClusterSize(500);
	//ec.setMaxClusterSize(25000);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::searchNeighbourOctreeOutsideRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,float resolution,float radius, pcl::PointXYZ* searchPoint)
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
		for(int i=0;i<cloudSource->points.size();++i)
		{
			if(std::find(pointIdxRadiusSearch.begin(),pointIdxRadiusSearch.end(),i)==pointIdxRadiusSearch.end())
			{
				if(cloudSource->points[i].x==0&&cloudSource->points[i].y==0&&cloudSource->points[i].z==0)
					continue;
				nonNullIndex.push_back(i);
			}
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
	/*if ( kdtree.radiusSearch (*searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
		for(int i=0;i<pointIdxRadiusSearch.size();++i)
		{
			if(cloudSource->points[pointIdxRadiusSearch[i]].x==0&&cloudSource->points[pointIdxRadiusSearch[i]].y==0&&cloudSource->points[pointIdxRadiusSearch[i]].z==0)
				continue;
			cloud_neighbours->points.push_back(cloudSource->points[pointIdxRadiusSearch[i]]);
		}
	}

	cloud_neighbours->width=cloud_neighbours->points.size();
	cloud_neighbours->height=1;
	cloud_neighbours->is_dense=true;
	cloud_neighbours->resize(cloud_neighbours->width);*/
	
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



//

bool PointCloud::getNearBlobs2( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr leftHandCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr rightHandCloud)
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
		if( i==0 ||dists[i]<smallestdist){
			ind=inds1[i];
			smallestdist=dists[i];
		}
	}

	smallestdist=std::sqrt(smallestdist);
	pt1=cloud->points[ind];

	
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
	NNN(cloud,&pt2,temp,150);//150
	if(!(findNearbyPts(cloud,temp,nearcent1)))
	return false;

	pcl::compute3DCentroid(handCloud,inds2,centroid1);
	pt2.x=centroid1(0);
	pt2.y=centroid1(1)-10;
	pt2.z=centroid1(2);
	NNN(cloud,&pt2,inds2,100);

	getSubCloud(cloud,inds2,cloudout);


	//rightHandCloud=cloudout;
	rightHandCloud->points.swap(cloudout->points);
	//*if(!foundarm)*/
	arm_center[0]=nearcent1;
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

	smallestdist=std::sqrt(smallestdist);
	pt1=cloud->points[ind];

	
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
	if(!(findNearbyPts(cloud,temp,nearcent1)))
	return false;

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

bool PointCloud::findNearbyPts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid)
{
	std::vector<int> inds(cloud->points.size(),1);
	std::vector<int>nearpts;
	std::vector<int>temp;
	for(int i=0;i<cloudpts.size();++i)
		inds[cloudpts[i]]=-1;
	for(int i=0;i<cloudpts.size();++i)
	{
		if(inds[cloudpts[i]]==-1)
		{
			NNN(cloud,&(cloud->points[cloudpts[i]]),temp,50);
			//NNN(cloud,&cloud.points[cloudpts[i]],temp,50);
			for(int j=0;j<temp.size();++j)
			{
				if(inds[temp[j]]==1)
				{
					nearpts.push_back(temp[j]);
					inds[temp[j]]=2;
				}
				else
				{
					/*if(temp[j]>=0)*/
					inds[temp[j]]=-2;
				}
					
			}
		}
	}
	if(nearpts.size())
	{
		//pcl::PointCloud<pcl::PointXYZ> handCloud=*cloud;
		pcl::compute3DCentroid(*cloud,nearpts,centroid);
	}
		
	else
		return false;
	return true;
}

bool PointCloud::findNearbyPts(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid)
{
	
	std::vector<int> inds(cloud->points.size(),1);
	std::vector<int>nearpts;
	std::vector<int>temp;
	for(int i=0;i<cloudpts.size();++i)
		inds[cloudpts[i]]=-1;
	for(int i=0;i<cloudpts.size();++i)
	{
		if(inds[cloudpts[i]]==-1)
		{
			NNN(cloud,&(cloud->points[cloudpts[i]]),temp,50);
			//NNN(cloud,&cloud.points[cloudpts[i]],temp,50);
			for(int j=0;j<temp.size();++j)
			{
				if(inds[temp[j]]==1)
				{
					nearpts.push_back(temp[j]);
					inds[temp[j]]=2;
				}
				else
					inds[temp[j]]=-2;
			}
		}
	}
	if(nearpts.size())
	{
		//pcl::PointCloud<pcl::PointXYZ> handCloud=*cloud;
		pcl::compute3DCentroid(*cloud,nearpts,centroid);
	}
		
	else
		return false;
	return true;
}

void PointCloud::getSubCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud)
{
	subCloud->width=subCloudIndex.size();
	subCloud->height=1;
	subCloud->resize(subCloud->width);

	for(int i=0;i<subCloudIndex.size();i++)
	{
		subCloud->points[i]=(cloudSource->points[subCloudIndex[i]]);
	}
}
void PointCloud::getSubCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr subCloud)
{
	subCloud->width=subCloudIndex.size();
	subCloud->height=1;
	subCloud->resize(subCloud->width);

	for(int i=0;i<subCloudIndex.size();i++)
	{
		subCloud->points[i]=(cloudSource->points[subCloudIndex[i]]);
	}
}
//performs radius search in a simplistic fashion

void PointCloud::getSubCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, std::vector<int> subCloudIndex,pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud,bool keep)
{
	
	if(keep==false)
	{
		std::vector<int> remainIndex;
		for(int i=0;i<cloudSource->points.size();++i)
		{
			if(std::find(subCloudIndex.begin(),subCloudIndex.end(),i)==subCloudIndex.end())
				remainIndex.push_back(i);
		}

		subCloud->width=remainIndex.size();
		subCloud->height=1;
		subCloud->resize(remainIndex.size());

		for(int i=0;i<remainIndex.size();++i)
		{
			subCloud->points[i]=cloudSource->points[remainIndex[i]];
		}
	}

	else
	{
		subCloud->width=subCloudIndex.size();
		subCloud->height=1;
		subCloud->resize(subCloudIndex.size());

		for(int i=0;i<subCloudIndex.size();i++)
		{
			subCloud->points[i]=cloudSource->points[subCloudIndex[i]];
		}
	}

	
}

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

void PointCloud::getEigens(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,int hand)
{
	pcl::PointCloud<pcl::PointXYZ> cloud=*handCloud;
	Eigen::Vector4f centroid;
	Eigen::Vector4f direction;
	Eigen::Vector4f armvector;

	Eigen::Vector3f eigen_values;
	Eigen::Matrix3f eigen_vectors;
	Eigen::Matrix3f cov;

	pcl::compute3DCentroid(cloud,centroid);
	distfromsensor=centroid.norm();
	pcl::computeCovarianceMatrixNormalized(cloud,centroid,cov);

	pcl::eigen33(cov,eigen_vectors,eigen_values);
	
	direction(0)=eigen_vectors (0, 2);
	direction(1)=eigen_vectors (1, 2);
	direction(2)=eigen_vectors (2, 2);
	
	if(hand==0)
	{
		/*	armvector(0)=arm_center[0]->x();
		armvector(1)=arm->y();
		armvector(2)=arm->z();*/
		armvector=arm_center[0];
	}
	if(hand==1)
	{
		armvector=arm_center[1];
	}



	flipvec(armvector,centroid,direction);

	handPoints[0]=centroid;
	handPoints[1]=centroid+direction;
	handDirection=direction;
}

void PointCloud::flipvec(const Eigen::Vector4f &palm, const Eigen::Vector4f &fcentroid,Eigen::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }

 void PointCloud::radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr handCloud,int nnthresh,double tol,int hand,pcl::PointCloud<pcl::PointXYZ>::Ptr palm,pcl::PointCloud<pcl::PointXYZ>::Ptr digits)
 {
	

	 vector<int> tempinds;
	 if(hand==0)
		 NNN(handCloud,&eigenToPclPoint(arm_center[0]),tempinds,100);
	 else
		 NNN(handCloud,&eigenToPclPoint(arm_center[1]),tempinds,100);

	 pcl::PointCloud<pcl::PointXYZ>::Ptr handWithOutArm(new  pcl::PointCloud<pcl::PointXYZ>);
	 getSubCloud(handCloud,tempinds,handWithOutArm,false);
	 /*if(handCloud->points.size()==0)
	 {
		 std::cout<<"000000000000000000000"<<std::endl;
	 }*/

	 pcl::PointCloud<pcl::PointXYZ> handPoints=*handWithOutArm;
	 Eigen::Vector4f handCenter;
	 
	 //////////method 1
	 pcl::compute3DCentroid(handPoints,handCenter);
	 pcl::PointXYZ searchCenter=eigenToPclPoint(handCenter);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr palmCloud=searchNeighbourOctreeRadius(handWithOutArm,30,45,&searchCenter);//30,45
	  pcl::PointCloud<pcl::PointXYZ>::Ptr digitsCloud=searchNeighbourOctreeOutsideRadius(handWithOutArm,30,45,&searchCenter);//30,45

	  
	/* palm->points.swap(palmCloud->points);
	 digits->points.swap(digitsCloud->points);*/
	  ///////////////////////////////
	  std::vector<int> inds,inds2,inds3;
	  std::vector<int> searchinds;
	  pcl::PointCloud<pcl::PointXYZ> potentialPoints=*digitsCloud;
	  SplitCloud2 sc2(potentialPoints,tol);
	  inds2.resize(potentialPoints.points.size(),-1);

	  int label;
	  for(int i=0;i<potentialPoints.points.size();++i)
	  {
		  if(inds2[i]==0)
			 continue;
		 sc2.NNN(&potentialPoints.points[i],searchinds,tol,false);
		 if(searchinds.size()>75)
		 {
			 inds.push_back(i);

			 if(searchinds.size()>90)//80//tol=20
				 label=0;
			 else 
				 label=1;
			 for(int j=0;j<searchinds.size();++j)
				 inds2[searchinds[j]]=label;
		 }
	  }

	  for(int i=0;i<potentialPoints.points.size();++i)
	  {
		  if(inds2[i]==-1)
			  inds3.push_back(i);
	  }

	  getSubCloud(digitsCloud,inds,palm,true);
	  getSubCloud(digitsCloud,inds3,digits,true);


	 //////////method 2
	// double t1,t2,t3;
	// std::vector<int> inds,inds2,inds3;
	// std::vector<int> searchinds;
	// //pcl::PointCloud<pcl::PointXYZ> handPoints=*handWithOutArm;
	// SplitCloud2 sc2(handPoints,tol);
	// inds2.resize(handPoints.points.size(),-1);

	// int min=9999;
	// int max=0;

	// int label;

	// for(int i=0;i<handPoints.points.size();++i)
	// {
	//	 if(inds2[i]==0)
	//		 continue;
	//	 sc2.NNN(&handPoints.points[i],searchinds,tol,false);

	//	 if(searchinds.size()>max)
	//		 max=searchinds.size();
	//	 if(searchinds.size()<min)
	//		 min=searchinds.size();
	//	 if(searchinds.size()>80)//65//tol=20
	//	 {
	//		 inds.push_back(i);

	//		 if(searchinds.size()>100)//80//tol=20
	//			 label=0;
	//		 else 
	//			 label=1;
	//		 for(int j=0;j<searchinds.size();++j)
	//			 inds2[searchinds[j]]=label;
	//	 }
	// }

	// for(int i=0;i<handPoints.points.size();++i)
	// {
	//	 if(inds2[i]==-1)
	//		 inds3.push_back(i);
	// }
	///*std::cout<<max<<std::endl;
	//std::cout<<min<<std::endl;
	// std::cout<<"next"<<std::endl;*/

	// getSubCloud(handWithOutArm,inds,palm,true);
	// getSubCloud(handWithOutArm,inds3,digits,true);
	
 }

 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloud::segFingers(pcl::PointCloud<pcl::PointXYZ>::Ptr digits,double clustertol,int mincluster)
 {
	 /*if(digits->points.size()==0)
		 return NULL;*/
	 std::vector<std::vector<int>> indclusts;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>clusters=euclideanClusterExtract(digits,clustertol,mincluster,1000);
	return clusters;
 }

 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::getColorPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr initialHand,int red,int green,int blue)
 {
	 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	 for(int i=0;i<initialHand->points.size();++i)
	 {
		 pcl::PointXYZRGBA point;
		 point.x=initialHand->points[i].x;
		 point.y=initialHand->points[i].y;
		 point.z=initialHand->points[i].z;
		 point.r=red;
		 point.g=green;
		 point.b=blue;
		 point.a=0;
		 colorCloud->points.push_back(point);
	 }
	 colorCloud->width=colorCloud->points.size();
	 colorCloud->height=1;
	 return colorCloud;
 }

 double PointCloud::checkFingerAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr fingerCloud)
 {
	 Eigen::Vector4f centroid;
	 Eigen::Vector4f direction;
	 Eigen::Vector3f eigen_values;
	 Eigen::Matrix3f eigen_vectors;
     Eigen::Matrix3f cov;
	 pcl::PointCloud<pcl::PointXYZ> finger=*fingerCloud;
	 pcl::compute3DCentroid(finger,centroid);
	 pcl::computeCovarianceMatrixNormalized(finger,centroid,cov);
	 pcl::eigen33(cov,eigen_vectors,eigen_values);
	 direction(0)=eigen_vectors (0, 2);
	 direction(1)=eigen_vectors (1, 2);
	 direction(2)=eigen_vectors (2, 2);
	 flipvec(handPoints[0],centroid,direction);
	 
	
	 double cos=direction.dot(handDirection);
	 return cos;
	/*if(cos>0)
		return true;
	else
		return false;
 }*/
 }

 double PointCloud::checkFingerDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr fingerCloud)
 {
	 Eigen::Vector4f centroid;
	 Eigen::Vector4f direction;
	 Eigen::Vector3f eigen_values;
	 Eigen::Matrix3f eigen_vectors;
     Eigen::Matrix3f cov;
	 pcl::PointCloud<pcl::PointXYZ> finger=*fingerCloud;
	 pcl::compute3DCentroid(finger,centroid);
	 pcl::computeCovarianceMatrixNormalized(finger,centroid,cov);
	 pcl::eigen33(cov,eigen_vectors,eigen_values);
	 direction(0)=eigen_vectors (0, 2);
	 direction(1)=eigen_vectors (1, 2);
	 direction(2)=eigen_vectors (2, 2);
	 flipvec(handPoints[0],centroid,direction);

	 pcl::PointCloud<pcl::PointXYZ>::Ptr neighbour=searchNeighbourKdTreeKNeighbour(fingerCloud,1,&pcl::PointXYZ(0,0,0));
	 if(neighbour->points.size()==0)
		 return 0;
	 Eigen::Vector4f nearestPoint;
	 nearestPoint(0)=neighbour->points[0].x;
	 nearestPoint(1)=neighbour->points[0].y;
	 nearestPoint(2)=neighbour->points[0].z;
	 Eigen::Vector4f distanceVector=nearestPoint-handPoints[0];
	 double distance=distanceVector(0)*distanceVector(0)+distanceVector(1)*distanceVector(1)+distanceVector(2)*distanceVector(2);
	 distance=std::pow(distance,0.5);
	 return distance;
	/* Eigen::Vector4f distanceVector=centroid-handPoints[0];
	 return distanceVector.norm();*/
 }

 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud::getFingerLine(pcl::PointCloud<pcl::PointXYZ>::Ptr fingerCloud)
 {
	 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fingerLine(new pcl::PointCloud<pcl::PointXYZRGBA>);
	 Eigen::Vector4f centroid;
	 Eigen::Vector4f direction;
	 Eigen::Vector3f eigen_values;
	 Eigen::Matrix3f eigen_vectors;
	 Eigen::Matrix3f cov;
	 pcl::PointCloud<pcl::PointXYZ> finger=*fingerCloud;
	 pcl::compute3DCentroid(finger,centroid);
	 pcl::computeCovarianceMatrixNormalized(finger,centroid,cov);
	 pcl::eigen33(cov,eigen_vectors,eigen_values);
	 direction(0)=eigen_vectors (0, 2);
	 direction(1)=eigen_vectors (1, 2);
	 direction(2)=eigen_vectors (2, 2);
	 flipvec(handPoints[0],centroid,direction);


	 pcl::PointXYZRGBA fingerCenter;
	 fingerCenter.x=centroid(0);
	 fingerCenter.y=centroid(1);
	 fingerCenter.z=centroid(2);
	 fingerCenter.r=100;
	 fingerCenter.g=50;
	 fingerCenter.b=200;
	 fingerLine->points.push_back(fingerCenter);

	 for(int i=0;i<50;++i)
	 {
		 pcl::PointXYZRGBA point;
		 point.x=fingerCenter.x+i*direction(0);
		 point.y=fingerCenter.y+i*direction(1);
		 point.z=fingerCenter.z+i*direction(2);
		 point.r=100;
		 point.g=50;
		 point.b=200;
		 fingerLine->points.push_back(point);
	 }

	 return fingerLine;
 }

 Eigen::Vector4f PointCloud::getHandCenter()
 {
	 return handPoints[0];
 }

 Eigen::Vector4f PointCloud::getHandDirection()
 {
	 return handDirection;
 }