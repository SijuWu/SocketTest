#include "StdAfx.h"
#include "PCLFinger.h"


PCLFinger::PCLFinger(Leap::Finger *leapFinger)
{
	this->leapFinger=leapFinger;

	this->x=leapFinger->tipPosition().x;
	this->y=leapFinger->tipPosition().y;
	this->z=leapFinger->tipPosition().z;

	this->r=255;
	this->g=255;
	this->b=255;
}


PCLFinger::~PCLFinger(void)
{
}

void PCLFinger::setColor(uint8_t r,uint8_t g,uint8_t b)
{
	this->r=r;
	this->g=g;
	this->b=b;
}