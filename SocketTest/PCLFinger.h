#pragma once
#include <pcl/point_types.h>
#include "Leap.h"
using namespace Leap;

class PCLFinger: public pcl::PointXYZRGB
{
public:
	PCLFinger(Leap::Finger *leapFinger);
	~PCLFinger(void);
	void setColor(uint8_t r,uint8_t g,uint8_t b);
private:
	Leap::Finger* leapFinger;
};

