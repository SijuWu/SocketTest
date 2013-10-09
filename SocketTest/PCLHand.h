#pragma once
#include <pcl/point_types.h>
#include "Leap.h"
#include "PCLFinger.h"
#include "stdafx.h"
using namespace Leap;

class PCLHand:  public pcl::PointXYZRGB
{
public:
	PCLHand(Leap::Hand* leapHand);
	~PCLHand(void);
	void setColor(uint8_t r,uint8_t g,uint8_t b);

	//std::vector<PCLFinger*> getFingers();

private:
	Leap::Hand* leapHand;
	//std::vector<PCLFinger*> pclFingers;
};

