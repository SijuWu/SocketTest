#pragma once
#include <stdlib.h> 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/features/feature.h>
#include <Eigen/StdVector>


class SplitCloud2
{
public:
	SplitCloud2(void);
	SplitCloud2(pcl::PointCloud<pcl::PointXYZ> &cloud, double tol);
	~SplitCloud2(void);
	 void fillMInds(pcl::PointCloud<pcl::PointXYZ> &cloud);
	 void initClouds(pcl::PointCloud<pcl::PointXYZ> &cloud);
	 int getMIndex(pcl::PointXYZ &pt);
	 int getIndex(pcl::PointXYZ &pt);
	 pcl::PointCloud<pcl::PointXYZ> * getCloud();
	 void NNN(pcl::PointXYZ* pt, std::vector<int> &inds, double radius, bool usesubset);

	double max_tol; //maximum tolerance that can be used in any algorithm and still maintain correctness
	double xdiv,ydiv,zdiv;
	double xdivs[8],ydivs[8],zdivs[8];
	double lx[64],ux[64],ly[64],uy[64],lz[64],uz[64];
	pcl::PointCloud<pcl::PointXYZ> clouds[64];

	 
	pcl::PointCloud<pcl::PointXYZ> *_cloud;
	std::vector< std::vector<int> > cinds;
	std::vector< std::vector<int> > minds;

	//for searching over a subset of the indices
	std::vector< std::vector<int> > subinds;
	std::vector<int>  subindmap;
};

