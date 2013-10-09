#include "StdAfx.h"
#include "PCLHand.h"


PCLHand::PCLHand(Leap::Hand* leapHand)
{
	this->leapHand=leapHand;

	this->x=leapHand->palmPosition().x;
	this->y=leapHand->palmPosition().y;
	this->z=leapHand->palmPosition().z;

	this->r=255;
	this->g=255;
	this->b=255;

	/*this->pclFingers.resize(0);*/
	/*for(int i=0;i<leapHand->fingers().count();++i)
	{
		PCLFinger finger=PCLFinger(&leapHand->fingers()[i]);
		this->pclFingers.push_back(&finger);
	}*/
}


PCLHand::~PCLHand(void)
{
}

void PCLHand::setColor(uint8_t r,uint8_t g,uint8_t b)
{
	this->r=r;
	this->g=g;
	this->b=b;
}

//std::vector<PCLFinger*> PCLHand::getFingers()
//{
//	return this->pclFingers;
//}