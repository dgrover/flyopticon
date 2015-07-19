#include <osg/Geode>
#include <osg/TexMat>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>

double screenHeight=800;//768;//1200;
double screenWidth=2560;//1366;//1920;
double xoffset = 1920;
double yoffset = 0;
double depth=0;
double camHorLoc=0;
double camVertLoc=0;
double distance=1;
osg::Vec3d up=osg::Vec3d(0,0,1); //up vector
std::string imageFileName="vert_stripes_40pix.bmp";
double transRate=0.3;
//2560*800






class TextureUpdateCallback: public osg::NodeCallback
{
private:
	osg::TexMat* texMat;
public:
	TextureUpdateCallback(osg::TexMat* tm)
	{
		texMat=tm;
	}

	virtual void operator()(osg::Node*, osg::NodeVisitor* nv)
	{
		if (!texMat)
		{
			return;
		}

		if (nv->getFrameStamp())
		{
			double currTime = nv->getFrameStamp()->getSimulationTime();
			float s=currTime*transRate;
			texMat->setMatrix(osg::Matrix::translate(s,0.0,0.0));
		}
	}
};







osg::Geode* createDisplay()
{
	osg::Geode* geode = new osg::Geode();
	osg::StateSet* stateset = new osg::StateSet();
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	osg::Image* image = new osg::Image();
	image = osgDB::readImageFile(imageFileName);


	if (image)
	{
		osg::Texture2D* texture = new osg::Texture2D(image);
		texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		osg::TexMat* texmat = new osg::TexMat;
		stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
		stateset->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
	}


	osg::TessellationHints* hints = new osg::TessellationHints;
	hints->setDetailRatio(0.8f);
	hints->setCreateTop(false);
	hints->setCreateBottom(false);
	hints->setCreateBackFace(false);
	hints->setCreateFrontFace(false);
	hints->setCreateNormals(false);    

	osg::Box* box = new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),screenWidth, screenWidth, screenHeight);
	osg::ShapeDrawable* cube = new osg::ShapeDrawable(box ,hints);
	cube->setUseDisplayList(false);
	cube->setStateSet( stateset );
	geode->setCullingActive(false);
	geode->addDrawable(cube);
	osg::TexMat* texmat = (osg::TexMat*)(stateset->getTextureAttribute(0,osg::StateAttribute::TEXMAT));
	geode->setUpdateCallback(new TextureUpdateCallback(texmat));
	return geode;
}




int main(int argc, char **argv)
{
	osgViewer::Viewer viewer;
	osg::Group* root = new osg::Group;
	root->addChild(createDisplay());
	viewer.setSceneData(root);
	bool doubleBuf=true;
	bool border=false;


	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = xoffset;
	traits->y = yoffset;
	traits->width = screenWidth;
	traits->height = screenHeight;
	traits->windowDecoration = border;
	traits->doubleBuffer = doubleBuf;
	traits->sharedContext = 0;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
	camera->setDrawBuffer(buffer);
	camera->setReadBuffer(buffer);

	osg::Matrixd camRot;
	osg::Matrixd camTrans;
	camRot.makeRotate(osg::DegreesToRadians(180.0), osg::Vec3(0,1,0));
	camTrans.makeTranslate(0.0,0.0,distance*2);


	viewer.addSlave(camera);
	viewer.getCamera()->setProjectionMatrixAsOrtho2D(-screenWidth/2, screenWidth/2, -screenHeight/2, screenHeight/2);
	viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up); //eyepos, center, up
	viewer.setCameraManipulator(NULL);

	while(!viewer.done())
	{
		viewer.frame();
	}

	return 0;
}