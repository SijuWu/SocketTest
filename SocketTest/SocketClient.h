#pragma once
#include <WinSock.h>

class SocketClient
{
private:
	SOCKET clientSocket;
	HWND hwnd;

public:
	SocketClient(void);
	~SocketClient(void);
	bool ConnectToHost(int PortNo,char* IPaddress);
	void CloseConnection();
	SOCKET getClientSocket();

};

