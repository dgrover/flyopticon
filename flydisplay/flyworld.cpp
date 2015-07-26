#include "stdafx.h"
#include "flyworld.h"

osg::ref_ptr<osg::Geode> cylNode = NULL;
bool visible = false;

class keyboardHandler : public osgGA::GUIEventHandler
{
public:
	virtual bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&);
};

bool keyboardHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	switch (ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::KEYDOWN) :
	{
		switch (ea.getKey())
		{
		case 0xFFBF:	//press F2 to show display
			visible = !visible;
			cylNode->setNodeMask(visible ? 0xffffffff : 0x0);
			break;
		
		default:
			return false;
			break;
		}
		return true;
	}
	default:
		return false;
		break;
	}
}

//FlyWorld::FlyWorld(char *imgFiles, char *sequence, char *settings, int w, int h, int x) :
//imageFiles(imgFiles), sequenceFile(sequence), displayFile(settings), viewWidth(w), viewHeight(2*h), xOffset(x),
//yOffset(0), cRadius(7.5 / 2.0), cHeight(6), defaultDistance(cRadius + 12.5), distance(defaultDistance), defaultCull(0),
//cull(defaultCull), camHorLoc(0), camVertLoc(cHeight*-0.5), depth(0)
//{
//	blackbgcolor = osg::Vec4(0, 0, 0, 1);	//black background
//		
//	up = osg::Vec3d(0, 0, 1);
//	setup();
//}

FlyWorld::FlyWorld(char *imgFiles, char *settings, int w, int h, int x) :
imageFiles(imgFiles), displayFile(settings), viewWidth(w), viewHeight(2 * h), xOffset(x),
yOffset(0), cRadius(7.5 / 2.0), cHeight(6), defaultDistance(cRadius + 12.5), distance(defaultDistance), defaultCull(0),
cull(defaultCull), camHorLoc(0), camVertLoc(cHeight*-0.5), depth(0)
{
	blackbgcolor = osg::Vec4(0, 0, 0, 1);	//black background

	up = osg::Vec3d(0, 0, 1);
	setup();
}


osgDB::DirectoryContents FlyWorld::getImages(std::string directory)
{
	osgDB::DirectoryContents images;

	if (osgDB::fileType(directory) == osgDB::DIRECTORY)
	{
		osgDB::DirectoryContents dc = osgDB::getSortedDirectoryContents(directory);

		for (osgDB::DirectoryContents::iterator itr = dc.begin(); itr != dc.end(); ++itr)
		{
			std::string filename = directory + "/" + (*itr);
			std::string ext = osgDB::getLowerCaseFileExtension(filename);
			if ((ext == "jpg") || (ext == "png") || (ext == "gif") || (ext == "rgb") || (ext == "bmp"))
			{
				images.push_back(filename);
			}
		}
	}

	else
	{
		images.push_back(directory);
	}

	return images;
}


osg::ref_ptr<osg::Geode> FlyWorld::createShapes()
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osgDB::DirectoryContents images = getImages(imageFiles);
	imageSequence = new osg::ImageSequence;
	imageSequence->setMode(osg::ImageSequence::Mode::PRE_LOAD_ALL_IMAGES);

	if (!images.empty())
	{
		for (osgDB::DirectoryContents::iterator itr = images.begin(); itr != images.end(); ++itr)
		{
			const std::string& filename = *itr;
			osg::ref_ptr<osg::Image> image = osgDB::readImageFile(filename);

			if (image.valid())
			{
				imageSequence->addImage(image.get());
			}
		}

		numImages = imageSequence->getNumImageData();
	}

	if (imageSequence)
	{
		osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(imageSequence.get());
		texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		osg::ref_ptr<osg::TexMat> texmat = new osg::TexMat;
		stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
		stateset->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
		texture->setUnRefImageDataAfterApply(true);
	}

	geode->setStateSet(stateset);
	geode->setCullingActive(true);
	osg::ref_ptr<osg::CullFace> cull = new osg::CullFace(osg::CullFace::Mode::BACK);
	stateset->setAttributeAndModes(cull, osg::StateAttribute::ON);
	osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
	hints->setDetailRatio(4.0f);
	hints->setCreateBackFace(false);
	hints->setCreateFrontFace(true);
	hints->setCreateNormals(false);
	hints->setCreateTop(false);
	hints->setCreateBottom(false);
	osg::ref_ptr<osg::Cylinder> cyl = new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.0f), cRadius, cHeight);
	cyl->setRotation(osg::Quat(0, osg::Vec3(0, 0, 1)));
	osg::ref_ptr<osg::ShapeDrawable> cylinder = new osg::ShapeDrawable(cyl, hints);
	cylinder->setUseDisplayList(false);
	geode->addDrawable(cylinder);
	return geode;
}


void FlyWorld::setDisplay()
{
	float number;
	std::ifstream file(displayFile, std::ios::in);

	distance = defaultDistance;
	cull = defaultCull;

	if (file.is_open())
	{
		if (file >> number)
		{
			distance = number;

			if (file >> number)
			{
				cull = number;
			}

			else
			{
				distance = defaultDistance;
				cull = defaultCull;
			}
		}

		file.close();
	}
}

//void FlyWorld::setSequence()
//{
//	float number;
//	std::ifstream file(sequenceFile, std::ios::in);
//
//	if (file.is_open())
//	{
//		while (file >> number)
//			sequence.push_back(number);
//
//		file.close();
//	}
//	else
//	{
//		for (unsigned int i = 0; i < numImages; i++)
//			sequence.push_back(1);
//	}
//}


void FlyWorld::setup()
{
	osg::setNotifyLevel(osg::NotifySeverity::ALWAYS);
	
	cylNode = createShapes();

	osg::ref_ptr<osg::Group> root = new osg::Group;
	//root->addChild(switchNode.get());
	root->addChild(cylNode.get());

	cylNode->setNodeMask(0x0);

	//setSequence();

	viewer.setSceneData(root);

	setDisplay();

	keyboardHandler* handler = new keyboardHandler();
	viewer.addEventHandler(handler);
	
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = xOffset;
	traits->y = yOffset;
	traits->width = viewWidth;
	traits->height = viewHeight;
	traits->windowDecoration = false;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	viewer.getCamera()->setGraphicsContext(gc.get());
	viewer.getCamera()->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
	viewer.getCamera()->setDrawBuffer(buffer);
	viewer.getCamera()->setReadBuffer(buffer);
	
	viewer.getCamera()->setClearColor(blackbgcolor);
	
	viewer.setCameraManipulator(NULL);
	viewer.getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	viewer.getCamera()->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);


	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = xOffset + viewWidth;
		traits->y = yOffset + 0;
		traits->width = viewWidth;
		traits->height = viewHeight;
		traits->windowDecoration = false;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::translate(0.0, 0.0, distance)*osg::Matrixd::rotate(osg::DegreesToRadians(-90.0), osg::Vec3(0, 1, 0))*osg::Matrixd::translate(0.0, 0.0, -1 * distance));
	}

	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = xOffset + viewWidth*2.0;
		traits->y = yOffset + 0;
		traits->width = viewWidth;
		traits->height = viewHeight;
		traits->windowDecoration = false;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::translate(0.0, 0.0, distance * 2)*osg::Matrixd::rotate(osg::DegreesToRadians(180.0), osg::Vec3(0, 1, 0)));
	}

	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = xOffset + viewWidth*3.0;
		traits->y = yOffset + 0;
		traits->width = viewWidth;
		traits->height = viewHeight;
		traits->windowDecoration = false;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::translate(0.0, 0.0, distance)*osg::Matrixd::rotate(osg::DegreesToRadians(90.0), osg::Vec3(0, 1, 0))*osg::Matrixd::translate(0.0, 0.0, -1 * distance));
	}

	viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc, distance, camVertLoc), osg::Vec3d(camHorLoc, depth, camVertLoc), up);
	//viewer.getCamera()->setProjectionMatrixAsPerspective(40.0, (float)viewWidth / (float)viewHeight, distance - cRadius, distance - cull);
	viewer.getCamera()->setProjectionMatrixAsPerspective(40.0, 1280.0 / (800.0*2), distance - cRadius, distance - cull);
}