#include <iostream>
#include "SerialPort.h"

#include <process.h> 
//#pragma comment( linker, "/subsystem:windows /entry:mainCRTStartup" )
int main()
{

	CSerialPort mySerialPort;
	//if (!mySerialPort.InitPort(1))
	
	if (!mySerialPort.openComPort(1, 115200, 'N', 8, 1, EV_RXCHAR)) {
	};
	mySerialPort.ReadData();
	/*
	if (!mySerialPort.OpenListenThread())
	{
		std::cout << "OpenListenThread fail !" << std::endl;
	}
	else
	{
		std::cout << "OpenListenThread success !" << std::endl;
	}
	*/
	//system("pause");
	return 0;

}
