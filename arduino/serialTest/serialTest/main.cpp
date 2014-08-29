#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>

// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
	Serial* SP = new Serial("COM3");    // adjust as needed

	if (SP->IsConnected())
		printf("connected");

	while(1)
	{
		SP->WriteData("1", 1);
		SP->WriteData("0", 1);
	}

	return 0;
}