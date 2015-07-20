// flydisplay.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	FlyWorld mov("images", "sequence.txt", "displaySettings.txt", 912, 1140, 1920);
	printf("%d images read [OK]\n", mov.numImages);

	while (!mov.viewer.done())
	{
		for (int i = 0; i < mov.numImages; i++)
		{
			for (int j = 0; j < mov.sequence[i]; j++)
			{
				double stime = ((double)i) / ((double)(mov.numImages - 1));
				mov.imageSequence->seek(stime);
				mov.viewer.frame();
			}
		}
	}

	return 0;
}

