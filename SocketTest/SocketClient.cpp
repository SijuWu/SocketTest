#include "StdAfx.h"
#include "SocketClient.h"


SocketClient::SocketClient(void)
{
}


SocketClient::~SocketClient(void)
{
}

bool SocketClient::ConnectToHost(int PortNo, char* IPAddress)
{
	WSADATA wsadata;

	int error=WSAStartup(0x0202,&wsadata);

	if(error)
		return false;

	if(wsadata.wVersion!=0x0202)
	{
		WSACleanup();
		return false;
	}

	SOCKADDR_IN target;

	target.sin_family=AF_INET;
	target.sin_port=htons(PortNo);
	target.sin_addr.S_un.S_addr=inet_addr(IPAddress);

	clientSocket=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
	if(clientSocket==INVALID_SOCKET)
	{
		return false;
	}

	if(connect(clientSocket,(SOCKADDR *)&target,sizeof(target))==SOCKET_ERROR)
	{
		return false;
	}
	else
		return true;
}

void SocketClient::CloseConnection()
{
	if(clientSocket)
		closesocket(clientSocket);
	WSACleanup();
}

SOCKET SocketClient::getClientSocket()
{
	return clientSocket;
}