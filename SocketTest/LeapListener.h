#pragma once
#include "Leap.h"
using namespace Leap;
class LeapListener:public Listener
{
public:
	LeapListener(void);
	~LeapListener(void);
	void onInit(const Controller&);
	void onConnect(const Controller&);
	void onDisconnect(const Controller&);
	void onExit(const Controller&);
	void onFrame(const Controller&);
	void onFocusGained(const Controller&);
	void onFocusLost(const Controller&);
};