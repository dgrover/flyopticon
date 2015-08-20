// flydisplay.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
using namespace std;

#define N 1

int _tmain(int argc, _TCHAR* argv[])
{
	FlyWorld mov("images", "displaySettings.txt", 912, 1140, 1920);
	printf("%d images read [OK]\n", mov.numImages);

	while (!mov.viewer.done())
	{
		for (int i = 0; i < mov.numImages; i++)
		{
			//for (int j = 0; j < mov.sequence[i]; j++)
			for (int j = 0; j < N; j++)
			{
				//osg::Timer_t startTick = osg::Timer::instance()->tick();
				
				double stime = ((double)i) / ((double)(mov.numImages - 1));
				mov.imageSequence->seek(stime);
				mov.viewer.frame();

				//osg::Timer_t endTick = osg::Timer::instance()->tick();
				//std::cout << "Completed in " << osg::Timer::instance()->delta_m(startTick, endTick) << std::endl;
			}
		}
	}

	return 0;
}

