#pragma once
#ifndef UDPSOCKET_H_
#define UDPSOCKET_H_

#include <Windows.h>
#include <winsock.h>
#pragma comment(lib, "ws2_32.lib")
using namespace std;
class UdpSocket
{
public:
	UdpSocket();
	~UdpSocket();
	void SocketCreate();
	void SocketClose();
	void SendData(char* data, int dataLen);

private:

	int iWsaStartup;
	int iWsaCleanup;
	WSADATA WinSockData;
	SOCKET UDPSocketClient;
	struct sockaddr_in UDPServer;
	int iSendto;
	int iBufferLen;
	int iUDPServerLen;
	int iCloseSocket;
	char Buffer[512] = "Hello From Client!";
};
extern UdpSocket udpSocket;
#endif
