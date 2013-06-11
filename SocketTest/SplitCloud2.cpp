#include "StdAfx.h"
#include "SplitCloud2.h"


SplitCloud2::SplitCloud2(void)
{
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Constructor.  Sets the cloud we search over, and the maximum radius search we will guarantee to be correct
* \param cloud the incoming point cloud
* \param tol the maximum radius search we will guarantee to be correct
*/
SplitCloud2::SplitCloud2(pcl::PointCloud<pcl::PointXYZ> &cloud, double tol){
	max_tol=tol;
	Eigen::Vector4f vec;
	pcl::compute3DCentroid(cloud,vec);
	xdiv=vec.x(); ydiv=vec.y(); zdiv=vec.z();
	fillMInds(cloud); //fill out divs
	initClouds(cloud);
}

SplitCloud2::~SplitCloud2(void)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//** \brief fill a list of indices that indicate which hexiquadrant the point is in
//* also, fills out the boundaries of the subdivisions
//* \param cloud the incoming point cloud
//*/
void SplitCloud2::fillMInds(pcl::PointCloud<pcl::PointXYZ> &cloud){
	minds.resize(8);
	for(int i=0;i<cloud.points.size();i++){
		minds[getMIndex(cloud.points[i])].push_back(i);
	}
	Eigen::Vector4f vec;
	for(int i=0;i<8;i++){
		pcl::compute3DCentroid(cloud,minds[i],vec);
		xdivs[i]=vec.x(); ydivs[i]=vec.y(); zdivs[i]=vec.z();
	}

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//** \brief splits the cloud into 64 overlapping clouds representing each hexiquadrant
//* \param cloud the incoming point cloud
//*/
void SplitCloud2::initClouds(pcl::PointCloud<pcl::PointXYZ> &cloud){
	pcl::PointXYZ minpt,maxpt;
	pcl::getMinMax3D(cloud,minpt,maxpt);

	double minx=minpt.x-100,maxx=maxpt.x+100,miny=minpt.y-100,maxy=maxpt.y+100,minz=minpt.z-100,maxz=maxpt.z+100; //maximum limits of cloud, with a little extra
	//lower and upper bounds on cloud with tolerance subdivisions:
	for(int i=0;i<64;i++){
		//set hard limits
		lx[i]=minx; ux[i]=maxx; ly[i]=miny; uy[i]=maxy; lz[i]=minz; uz[i]=maxz;

		//apply first layer of limits:
		if(i%2 >= 1) //x bigger than xdiv
			lx[i] = xdiv - max_tol;
		else
			ux[i] = xdiv + max_tol;

		if(i%4 >= 2) //y bigger than xdiv
			ly[i] = ydiv - max_tol;
		else
			uy[i] = ydiv + max_tol;

		if(i%8 >= 4) //z bigger than xdiv
			lz[i] = zdiv - max_tol;
		else
			uz[i] = zdiv + max_tol;

		//now apply second layer of limits:
		if(i%16 >= 8) //x bigger than xdiv
			lx[i] = std::max(xdivs[i%8] - max_tol, lx[i]);
		else
			ux[i] = std::min(xdivs[i%8] + max_tol, ux[i]);
		if(i%32 >= 16) //y bigger than xdiv
			ly[i] = std::max(ydivs[i%8] - max_tol, ly[i]);
		else
			uy[i] = std::min(ydivs[i%8] + max_tol, uy[i]);
		if(i%64 >= 32) //z bigger than xdiv
			lz[i] = std::max(zdivs[i%8] - max_tol, lz[i]);
		else
			uz[i] = std::min(zdivs[i%8] + max_tol, uz[i]);

	}


	cinds.resize(64);
	//  clouds.resize(64);
	for(int i=0;i<cloud.points.size();i++){
		for(int index=0;index<64;index++){
			if (cloud.points[i].x >= lx[index] && cloud.points[i].x <= ux[index] && cloud.points[i].y >= ly[index] && cloud.points[i].y <= uy[index] && cloud.points[i].z >= lz[index] && cloud.points[i].z <= uz[index] )
				cinds[index].push_back(i);
		}
	}
	_cloud=&cloud;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//** \brief returns the index of the first split (0-8)
//* \param pt the point we are investigating
//* \return octant number
//*/
int SplitCloud2::getMIndex(pcl::PointXYZ &pt){
	int ret=0;
	if(pt.z > zdiv) ret +=4;
	if(pt.y > ydiv) ret +=2;
	if(pt.x > xdiv) ret +=1;
	return ret;
}

 pcl::PointCloud<pcl::PointXYZ> * SplitCloud2::getCloud()
 {
	 return _cloud;
 }


    //performs radius search in a simplistic fashion
   //usesubset: if UseInds() has been called, then use the inds.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief The main function of SplitCloud2 - does a radius search, returning indices of the original cloud
	* \param pt the point to search around
	* \param inds a vector of indices of the original cloud that gets filled out with the results of the search
	* \param radius the search radius
	* \param usesubset if UseInds() has been called, then only search over those points
	*/
   void SplitCloud2::NNN(pcl::PointXYZ* pt, std::vector<int> &inds, double radius, bool usesubset=false)
   {
     //check if usesubinds is a bad idea:
     if(usesubset && !subindmap.size()){
        usesubset=false;
     }

     int index=getIndex(*pt);

     inds.clear();
     double r2=radius*radius;
	 double smallerrad=radius/std::pow(3,0.5);
     double diffx,diffy,diffz;

     //rather than calling an if statement for every point in the cloud, just duplicate the code.
     //this does speed up the code a bit...
     if(!usesubset){
        for(int i=0;i<cinds[index].size(); i++){
            //find the distance between the cloud point and the reference in the three dimensions:
          diffx=fabs(_cloud->points[cinds[index][i]].x - pt->x);
          diffy=fabs(_cloud->points[cinds[index][i]].y - pt->y);
          diffz=fabs(_cloud->points[cinds[index][i]].z - pt->z);
          //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
          if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
              //if the point is also in a cube whose circumradius is the search radius, the point is close enough:
            if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
               inds.push_back(cinds[index][i]);
            else//between the boxes: check for actual distance
               if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
                 inds.push_back(cinds[index][i]);
          }//endif in outer box
        }//endfor all points in cloud
     }
     else{
        for(int i=0;i<subinds[index].size(); i++){
            //find the distance between the cloud point and the reference in the three dimensions:
          diffx=fabs(_cloud->points[subindmap[subinds[index][i]]].x - pt->x);
          diffy=fabs(_cloud->points[subindmap[subinds[index][i]]].y - pt->y);
          diffz=fabs(_cloud->points[subindmap[subinds[index][i]]].z - pt->z);
          //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
          if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
              //if the point is also in a cube whose circumradius is the search radius, the point is close enough:
            if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
               inds.push_back(subinds[index][i]);
            else//between the boxes: check for actual distance
               if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
                 inds.push_back(subinds[index][i]);
          }//endif in outer box
        }//endfor all points in cloud
     }

   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief returns the full index of the point
* This is called when searching the cloud, so we don't care about buffer zones
 * \param pt the point we are investigating
 * \return hexiquadrant number
 */
   int SplitCloud2::getIndex(pcl::PointXYZ &pt){
     int ret=0,index;
     if(pt.z > zdiv) ret +=4;
     if(pt.y > ydiv) ret +=2;
     if(pt.x > xdiv) ret +=1;
     index=ret;
     if(pt.z > zdivs[index]) ret +=32;
     if(pt.y > ydivs[index]) ret +=16;
     if(pt.x > xdivs[index]) ret +=8;
     return ret;
   }