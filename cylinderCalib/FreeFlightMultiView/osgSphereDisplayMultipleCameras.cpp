#include <osg/TexMat>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>

double screenWidth=1366;//800;
double screenHeight=768;//1580;

	int width=screenWidth/3;//800;
	int height=screenWidth/3;//1580;

 int xoffset = 0;//;1920;
    int yoffset = (screenHeight-height)/2;//-150;


osg::Vec4 backgroundColor = osg::Vec4(0, 0, 0, 1);

double diam=3.14159*2;
double depth=0;
double rS= 0.01*diam; //rotation speed
double camHorLoc=0;
double camVertLoc=0;
double distance=19.25;
osg::Vec3d up=osg::Vec3d(0,0,1); //up vector

std::string imageFileName="numberline.gif";//"vert_stripe.bmp";



class TextureUpdateCallback : public osg::NodeCallback
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
				float s=currTime*rS;
				texMat->setMatrix(osg::Matrix::translate(s,0.0,0.0));
			}
		}
};










osg::Geode* createShapes()
{
    osg::Geode* geode = new osg::Geode();
    osg::StateSet* stateset = new osg::StateSet();
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	osg::Image* image = new osg::Image();
	 image = osgDB::readImageFile(imageFileName);
	
    if (image)
    {
		osg::Texture2D* texture = new osg::Texture2D(image);
		texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		osg::TexMat* texmat = new osg::TexMat;
		stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
		stateset->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
		texture->setUnRefImageDataAfterApply(true);
    }
    
    geode->setStateSet( stateset );
	geode->setCullingActive(true);
    
    float radius = 4.8;//5; //0.8f;
    
   osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(3.0f);


   hints->setCreateBackFace(true);
hints->setCreateFrontFace(false);
hints->setCreateNormals(false);
hints->setCreateTop(false);
hints->setCreateBottom(false);
osg::Sphere* sph = new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),radius);

osg::ShapeDrawable* sphere = new osg::ShapeDrawable(sph ,hints);
  sphere->setUseDisplayList(false);

    geode->addDrawable(sphere);
   
	osg::TexMat* texmat = (osg::TexMat*)(stateset->getTextureAttribute(0,osg::StateAttribute::TEXMAT));	
	geode->setUpdateCallback(new TextureUpdateCallback(texmat));
    return geode;
}

int main( int argc, char **argv )
{
// construct the viewer.
	osgViewer::Viewer viewer;


 // create the model
    osg::Group* root = new osg::Group;
    root->addChild( createShapes() );


    viewer.setSceneData( root );
//osg::ref_ptr<osg::Camera> myCam = new osg::Camera;
//myCam->setClearColor(backgroundColor); // black background
	// set dimensions of the view volume
	//myCam->setProjectionMatrixAsPerspective(fovy, 3.0 / 3.0, 0.1, 3000);
	//viewer.setCamera(myCam); // attach camera to the viewer
   


    // front
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset + 0;
        traits->y = yoffset + 0;
        traits->width = width;
        traits->height = height;
        traits->windowDecoration = false;
        traits->doubleBuffer = true;
        traits->sharedContext = 0;

        osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext(gc.get());
        camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
    }
    
    // back
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset + width*2;
        traits->y = yoffset + 0;
        traits->width = width;
        traits->height = height;
        traits->windowDecoration = false;
        traits->doubleBuffer = true;
        traits->sharedContext = 0;

        osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext(gc.get());
        camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);


osg::Matrixd camRot;
osg::Matrixd camTrans1;
osg::Matrixd camTrans2;

camTrans1.makeTranslate(0.0,0.0,distance);
camRot.makeRotate(osg::DegreesToRadians(120.0), osg::Vec3(0,1,0));
camTrans2.makeTranslate(0.0,0.0,-1*distance);




//viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::translate(0.0,0.0,distance*2));
	viewer.addSlave(camera.get(), osg::Matrixd(),camTrans1*camRot*camTrans2);
	//viewer.addSlave(camera.get(), osg::Matrixd(),osg::Matrixd::rotate(osg::DegreesToRadians(180.0), osg::Vec3(0,1,0)).translate(0.0,0.0,distance*2));
    }







//right side
	
 {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset+width;
        traits->y = yoffset;
        traits->width = width;
        traits->height = height;
        traits->windowDecoration = false;
        traits->doubleBuffer = true;
        traits->sharedContext = 0;

        osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext(gc.get());
        camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);

osg::Matrixd camRot;
osg::Matrixd camTrans1;
osg::Matrixd camTrans2;
camTrans1.makeTranslate(0.0,0.0,distance);
camRot.makeRotate(osg::DegreesToRadians(-120.0), osg::Vec3(0,1,0));
camTrans2.makeTranslate(0.0,0.0,-1*distance);

//viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::translate(0.0,0.0,distance*2));
	viewer.addSlave(camera.get(), osg::Matrixd(),camTrans1*camRot*camTrans2);
	//viewer.addSlave(camera.get(), osg::Matrixd(),osg::Matrixd::rotate(osg::DegreesToRadians(180.0), osg::Vec3(0,1,0)).translate(0.0,0.0,distance*2));
    }







viewer.getCamera()->setClearColor(backgroundColor); // black background

viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up);


 
	viewer.setCameraManipulator(NULL);

	while(!viewer.done())
	{


		viewer.frame();
	}
	return 0;
}