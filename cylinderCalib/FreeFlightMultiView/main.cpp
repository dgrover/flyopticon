
#include <osg/Notify>
#include <osg/TextureRectangle>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/TexMat>
#include <osg/Group>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/Math>
//#include <osgSim>

double imageWidth=800;
double imageHeight=1280;

double fovy=30;

double boxHorMin=0;
double boxVertMin=0;
double boxHorMax=boxHorMin + imageWidth;
double boxVertMax=boxVertMin + imageHeight;

osg::Vec4 backgroundColor = osg::Vec4(0, 0, 0, 1);

double numBoxes=1;
double depth=0;
double xS=0.0; //speed of camera
double yS=0;
double zS=0.0;
double rS=0; //rotation speed
//double camHorLoc=boxHorMin+imageWidth/2*numBoxes; //location of camera
double camHorLoc=0;
double camVertLoc=0;
double distance=13.5;
osg::Vec3d up=osg::Vec3d(0,0,1); //up vector

std::string imageFileName="verticalLine.jpg";

//	setViewMatrixAsLookAt(osg::Vec3d(move,distance,camVertLoc+yS*currTime), osg::Vec3d(move,0.0,camVertLoc+yS*currTime), up);



  /*class rootDataType : public osg::Referenced
    {
   public:
    rootDataType::rootDataType(osg::Node* n)
    {
       rotation = 0;

       findNodeVisitor findCylinder("cylinder"); 
       n->accept(findCylinder);
       cylinderNode = 
		   dynamic_cast <osg::MatrixTransform> (findCylinder.getFirst());
    }
	 
	     void rootDataType::updateCylinderRotation()
    { 
       rotation += 0.01;
       cylinderNode->setCurrentHPR( osg::Vec3(rotation,0,0) );
    }
      
    protected:
       osgSim::Transform* cylinderNode;
       double rotation; // (radians)  
    };*/



  /* class GeodeUpdateCallback : public osg::NodeCallback 
    {
    public:
       virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
       {
          //osg::ref_ptr<tankDataType> tankData = 
            // dynamic_cast<tankDataType*> (node->getUserData() );
		  osg::Geode* geode = static_cast<osg::Geode*>( node ); 
		  geode->getDrawable(0);
          //root->getChild(0);
          traverse(node, nv); 
       }
    };*/

/*class CylinderUpdateCallback : public osg::Drawable::UpdateCallback 
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
osg::Quat* quat = new osg::Quat(30*currTime, *rotationVec);

osg::ShapeDrawable * geom = dynamic_cast< osg::ShapeDrawable * >( node ) ; 

osg::Shape* cyl = geom->getShape();
((osg::Cylinder*)cyl)->setRotation(*quat);
geom->setShape(cyl);


} 


}; 
*/




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
osg::Quat* quat = new osg::Quat(3.14159*2*currTime, *rotationVec);


osg::Geode * geode = dynamic_cast< osg::Geode * >( node ); 
osg::Drawable* drawable = geode->getDrawable(0);
//osg::ShapeDrawable * geom = dynamic_cast< osg::ShapeDrawable * >( node ) ; 

osg::Shape* cyl = ((osg::ShapeDrawable*) drawable)->getShape();
((osg::Cylinder*)cyl)->setRotation(*quat);
((osg::ShapeDrawable*) drawable)->setShape(cyl);


} 
}; 





/*
class CameraUpdateCallback : public osg::NodeCallback
{
public:
	CameraUpdateCallback(
		double delay = 0.0001):


	_delay(delay),
		_prevTime(0.0),
		timeToSubtract(0.0)
	{
	}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::Camera* cam = static_cast<osg::Camera*>( node ); 
		double move;

		if (nv->getFrameStamp()) {
			double currTime = nv->getFrameStamp()->getSimulationTime();
			//if (currTime - _prevTime > _delay) {

//use timeToSubtract..........................
//boxHorMin+imageWidth/2*3 switching point to boxHorMin+imageWidth/2 (or camHorLoc)

				//boxHorMin+imageWidth/2*numBoxes is middle of boxes
				move=camHorLoc+xS*(currTime-timeToSubtract);
if(move>=boxHorMin+imageWidth/2*3)
{
	
timeToSubtract=currTime;
move=camHorLoc;
}
//cam->setProjectionMatrixAsPerspective(30, 3.0 / 3.0, 0.1, 3000);
//cam->setProjectionMatrix(osg::Matrix::ortho(-1,1,-1,1,-1, 1));
				//cam->setViewMatrixAsLookAt(osg::Vec3d(move,distance,camVertLoc+yS*currTime), osg::Vec3d(move,0.0,camVertLoc+yS*currTime), up);
cam->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc+yS*currTime), osg::Vec3d(move,depth+zS*currTime,camVertLoc+yS*currTime), up);
//makeRotate (value_type angle, const Vec3f &axis)
osg::Matrixd cameraRotation;
osg::Matrixd cameraTrans;
cameraRotation.makeRotate(
			osg::DegreesToRadians(rS*currTime), osg::Vec3(0,1,0), // roll
          osg::DegreesToRadians(0.0), osg::Vec3(1,0,0) , // pitch
          osg::DegreesToRadians( 0.0), osg::Vec3(0,0,1) ); // heading 

 cameraTrans.makeTranslate( 0,0,15 );
cam->setViewMatrix(cam->getViewMatrix()*cameraRotation);


				_prevTime = currTime;
			}
		//}
		std::cout<<"Camera update callback - pre traverse"<<node<<std::endl;
		traverse(node,nv);
		std::cout<<"Camera update callback - post traverse"<<node<<std::endl;
	}
private:
	double _delay;
	double _prevTime;
	double  timeToSubtract;

};
*/









osg::Geode* createShapes()
{
    osg::Geode* geode = new osg::Geode();

    // ---------------------------------------
    // Set up a StateSet to texture the objects
    // ---------------------------------------
    osg::StateSet* stateset = new osg::StateSet();
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	osg::Image* image = new osg::Image();
	 image = osgDB::readImageFile(imageFileName);
	
    if (image)
    {
		
	osg::Texture2D* texture = new osg::Texture2D;
	texture->setImage(image);
	stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    }
    
    geode->setStateSet( stateset );
	geode->setCullingActive(false);
    
    float radius = 4.8;//5; //0.8f;
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
//cylinder->setUpdateCallback(new CylinderUpdateCallback());
    geode->addDrawable(cylinder);
   
    geode->setUpdateCallback(new GeodeUpdateCallback());
    return geode;
}

int main( int argc, char **argv )
{
// construct the viewer.
	osgViewer::Viewer viewer;


 // create the model
    osg::Group* root = new osg::Group;
    root->addChild( createShapes() );

   // root->setUpdateCallback(new RootUpdateCallback());
    // add model to viewer.
    viewer.setSceneData( root );
//osg::ref_ptr<osg::Camera> myCam = new osg::Camera;
//myCam->setClearColor(backgroundColor); // black background
	// set dimensions of the view volume
	//myCam->setProjectionMatrixAsPerspective(fovy, 3.0 / 3.0, 0.1, 3000);
	//viewer.setCamera(myCam); // attach camera to the viewer
   
//return viewer.run();
	//viewer.getCamera()->setUpdateCallback(new CameraUpdateCallback());




  int xoffset = 1920;
    int yoffset = 0;
	int width=800;
	int height=1280;

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












//walkthrough of fly flying around inside black and white striped cylinder,black floor, navigating around inside cylinder (ie fps)