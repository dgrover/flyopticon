#include <osg/TexMat>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>

double imageWidth=1366;//800;
double imageHeight=768;//1580;
 int xoffset = 0;//;1920;
    int yoffset = 0;//-150;
	int width=imageWidth/4;//800;
	int height=imageHeight;//1580;


double fovy=30;

double boxHorMin=0;
double boxVertMin=0;
double boxHorMax=boxHorMin + imageWidth;
double boxVertMax=boxVertMin + imageHeight;

osg::Vec4 backgroundColor = osg::Vec4(0, 0, 0, 1);

double diam=3.14159*2;
double numBoxes=1;
double depth=0;

double rS= 0.1*diam; //rotation speed
//double camHorLoc=boxHorMin+imageWidth/2*numBoxes; //location of camera
double camHorLoc=0;
double camVertLoc=0;
double distance=13.5;
osg::Vec3d up=osg::Vec3d(0,0,1); //up vector

std::string imageFileName="numberline.gif";//"vert_stripe.bmp";

class GeodeUpdateCallback : public osg::NodeCallback 
{ 
public : 

void operator()( osg::Node * node , osg::NodeVisitor * nv) 
{ 
	double currTime;
if (nv->getFrameStamp())
{
			 currTime= nv->getFrameStamp()->getSimulationTime();
}
osg::Vec3f* rotationVec=new osg::Vec3(0,0,1);
osg::Quat* quat = new osg::Quat(rS*currTime, *rotationVec);


osg::Geode * geode = dynamic_cast< osg::Geode * >( node ); 
osg::Drawable* drawable = geode->getDrawable(0);
//osg::ShapeDrawable * geom = dynamic_cast< osg::ShapeDrawable * >( node ) ; 

osg::Shape* cyl = ((osg::ShapeDrawable*) drawable)->getShape();
((osg::Cylinder*)cyl)->setRotation(*quat);
((osg::ShapeDrawable*) drawable)->setShape(cyl);


} 
}; 





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
		texture->setUnRefImageDataAfterApply(true);
    }
    
    geode->setStateSet( stateset );
	geode->setCullingActive(false);
    
    float radius = 3;//5; //0.8f;
    float height = 3.8;//3.6; //5.02654824574*8/12;
    
   osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(0.8f);


   hints->setCreateBackFace(true);
hints->setCreateFrontFace(false);
hints->setCreateNormals(false);
hints->setCreateTop(false);
hints->setCreateBottom(false);
osg::Vec3f* rotationVec=new osg::Vec3(0,0,1);
osg::Quat* quat = new osg::Quat(30, *rotationVec);
osg::Cylinder* cyl = new osg::Cylinder(osg::Vec3(0.0f,0.0f,0.0f),radius,height);
cyl->setRotation(*quat);
osg::ShapeDrawable* cylinder = new osg::ShapeDrawable(cyl ,hints);
  cylinder->setUseDisplayList(false);
    geode->addDrawable(cylinder);
   
    geode->setUpdateCallback(new GeodeUpdateCallback());

	//osg::TexMat* texmat = (osg::TexMat*)(stateset->getTextureAttribute(0,osg::StateAttribute::TEXMAT));	
	//geode->setUpdateCallback(new TextureUpdateCallback(texmat));
    return geode;
}

int main( int argc, char **argv )
{
	osgViewer::Viewer viewer;


    osg::Group* root = new osg::Group;
    root->addChild( createShapes() );


    viewer.setSceneData( root );




  /////

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
		//camera->setClearColor(backgroundColor); // black background
		//camera->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up);
	// set dimensions of the view volume
	//camera->setProjectionMatrixAsPerspective(fovy, 3.0 / 3.0, 0.1, 3000);

        // add this slave camera to the viewer, with a shift left of the projection matrix
       // viewer.addSlave(camera.get(), osg::Matrixd::translate(1.0,0.0,0.0), osg::Matrixd());
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
			//camera->setClearColor(backgroundColor); // black background
	// set dimensions of the view volume
//	camera->setProjectionMatrixAsPerspective(fovy, 3.0 / 3.0, 0.1, 3000);
//camera->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up);
        // add this slave camera to the viewer, with a shift right of the projection matrix
      //  viewer.addSlave(camera.get(), osg::Matrixd::translate(-1.0,0.0,0.0), osg::Matrixd()); //original

osg::Matrixd camRot;
osg::Matrixd camTrans;
camRot.makeRotate(osg::DegreesToRadians(180.0), osg::Vec3(0,1,0));
camTrans.makeTranslate(0.0,0.0,distance*2);

//viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::translate(0.0,0.0,distance*2));
	viewer.addSlave(camera.get(), osg::Matrixd(),camTrans*camRot);
	//viewer.addSlave(camera.get(), osg::Matrixd(),osg::Matrixd::rotate(osg::DegreesToRadians(180.0), osg::Vec3(0,1,0)).translate(0.0,0.0,distance*2));
    }


//left side
	
 {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset+width*3;
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
			//camera->setClearColor(backgroundColor); // black background
	// set dimensions of the view volume
//	camera->setProjectionMatrixAsPerspective(fovy, 3.0 / 3.0, 0.1, 3000);
//camera->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up);
        // add this slave camera to the viewer, with a shift right of the projection matrix
      //  viewer.addSlave(camera.get(), osg::Matrixd::translate(-1.0,0.0,0.0), osg::Matrixd()); //original

osg::Matrixd camRot;
osg::Matrixd camTrans1;
osg::Matrixd camTrans2;
camTrans1.makeTranslate(0.0,0.0,distance);
camRot.makeRotate(osg::DegreesToRadians(90.0), osg::Vec3(0,1,0));
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
			//camera->setClearColor(backgroundColor); // black background
	// set dimensions of the view volume
//	camera->setProjectionMatrixAsPerspective(fovy, 3.0 / 3.0, 0.1, 3000);
//camera->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up);
        // add this slave camera to the viewer, with a shift right of the projection matrix
      //  viewer.addSlave(camera.get(), osg::Matrixd::translate(-1.0,0.0,0.0), osg::Matrixd()); //original

osg::Matrixd camRot;
osg::Matrixd camTrans1;
osg::Matrixd camTrans2;
camTrans1.makeTranslate(0.0,0.0,distance);
camRot.makeRotate(osg::DegreesToRadians(-90.0), osg::Vec3(0,1,0));
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