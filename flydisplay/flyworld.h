#ifndef FLYWORLD_H
#define FLYWORLD_H

#include <osg/TexMat>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osg/CullFace>
#include <osgGA/GUIEventHandler>
#include <osg/Timer>

#include <osg/ImageSequence>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
//#include <osg/Switch>


class FlyWorld
{
private:

	int xOffset;
	int yOffset;
	int viewWidth;
	int viewHeight;
	float cRadius;
	float cHeight;
	double defaultDistance;
	double distance;
	double defaultCull;
	double cull;
	double camHorLoc;
	double camVertLoc;
	double depth;
	
	osg::Vec4 blackbgcolor;
	
	osg::Vec3d up;
	
	const char* imageFiles;
	const char* displayFile;
	//const char* sequenceFile;


	static osgDB::DirectoryContents getImages(std::string directory);
	osg::ref_ptr<osg::Geode> createShapes();
	void setDisplay();
	//void setSequence();
	void setup();

public:
	osg::ref_ptr<osg::ImageSequence> imageSequence;
	//std::vector<int> sequence;
	unsigned int numImages;
	osgViewer::Viewer viewer;

	//FlyWorld(char *imgFiles, char *sequence, char *settings, int w, int h, int x);
	FlyWorld(char *imgFiles, char *settings, int w, int h, int x);
};

#endif