#include "UdpSocket.h"
#include <iostream>
using namespace std;
#define PORTNUM 60111
UdpSocket udpSocket;

UdpSocket::UdpSocket()
{
	iWsaStartup = WSAStartup(MAKEWORD(2, 2), &WinSockData);

	if (iWsaStartup != 0)
	{
		cout << "WSAStartUP Failed!" << endl;
	}
	cout << "WSAStartUP Success!" << endl;
}


UdpSocket::~UdpSocket()
{
}

void UdpSocket::SocketCreate() {
	UDPSocketClient = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (UDPSocketClient == INVALID_SOCKET) {
		cout << "Socket Creation Falied & Error No->" << WSAGetLastError() << endl;
		return;
	}
	cout << "Socket Creation Success" << endl;
	UDPServer.sin_family = AF_INET;
	UDPServer.sin_addr.s_addr = inet_addr("127.0.0.1");
	UDPServer.sin_port = htons(PORTNUM);
}

void UdpSocket::SocketClose() {
	iCloseSocket = closesocket(UDPSocketClient);
	if (iCloseSocket == SOCKET_ERROR)
	{
		cout << "Socket Closing Failed & Error No->" << WSAGetLastError() << endl;
	}
	cout << "Socket Closing Success!" << endl;
}

void UdpSocket::SendData(char* data, int dataLen) {
	iSendto = sendto(
		UDPSocketClient,
		data,
		dataLen,
		MSG_DONTROUTE,
		(SOCKADDR*)&UDPServer,
		sizeof(UDPServer)
	);
	if (iSendto == SOCKET_ERROR)
	{
		cout << "Sending Data Falied & Error No->" << WSAGetLastError() << endl;
	}
	cout << "Sending Data Success!" << endl;
}