
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

#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/Math>


double imageWidth=1200;
double imageHeight=800;

double fovy=30;

double boxHorMin=0;
double boxVertMin=0;
double boxHorMax=boxHorMin + imageWidth;
double boxVertMax=boxVertMin + imageHeight;

osg::Vec4 backgroundColor = osg::Vec4(1, 1, 1, 1);

double numBoxes=1;
double depth=0;
double xS=0.0; //speed of camera
double yS=0;
double zS=0.0;
double rS=1; //rotation speed
//double camHorLoc=boxHorMin+imageWidth/2*numBoxes; //location of camera
double camHorLoc=0;
double camVertLoc=0;
double distance=-0.1;
osg::Vec3d up=osg::Vec3d(0,0,1); //up vector

std::string imageFileName="color2.jpg";

//	setViewMatrixAsLookAt(osg::Vec3d(move,distance,camVertLoc+yS*currTime), osg::Vec3d(move,0.0,camVertLoc+yS*currTime), up);







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
    
    float radius = 0.8f;
    float height = 1.0f;
    
   osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(1.0f);


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
  
    //geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),radius),hints));
   // geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(2.0f,0.0f,0.0f),2*radius),hints));
   // geode->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(4.0f,0.0f,0.0f),radius,height),hints));
    geode->addDrawable(cylinder);
   // geode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(8.0f,0.0f,0.0f),radius,height),hints));

  /*  osg::HeightField* grid = new osg::HeightField;
    grid->allocateGrid(38,39);
    grid->setXInterval(0.28f);
    grid->setYInterval(0.28f);
    
    for(unsigned int r=0;r<39;++r)
    {
	for(unsigned int c=0;c<38;++c)
	{
	    grid->setHeight(c,r,vertex[r+cf*39][2]);
	}
    }
    geode->addDrawable(new osg::ShapeDrawable(grid));*/
    
   /* osg::ConvexHull* mesh = new osg::ConvexHull;
    osg::Vec3Array* vertices = new osg::Vec3Array(4);
    (*vertices)[0].set(9.0+0.0f,-1.0f+2.0f,-1.0f+0.0f);
    (*vertices)[1].set(9.0+1.0f,-1.0f+0.0f,-1.0f+0.0f);
    (*vertices)[2].set(9.0+2.0f,-1.0f+2.0f,-1.0f+0.0f);
    (*vertices)[3].set(9.0+1.0f,-1.0f+1.0f,-1.0f+2.0f);
    osg::UByteArray* indices = new osg::UByteArray(12);
    (*indices)[0]=0;
    (*indices)[1]=2;
    (*indices)[2]=1;
    (*indices)[3]=0;
    (*indices)[4]=1;
    (*indices)[5]=3;
    (*indices)[6]=1;
    (*indices)[7]=2;
    (*indices)[8]=3;
    (*indices)[9]=2;
    (*indices)[10]=0;
    (*indices)[11]=3;*/
   // mesh->setVertices(vertices);
   // mesh->setIndices(indices);
   // geode->addDrawable(new osg::ShapeDrawable(mesh));
    
    return geode;
}

int main( int argc, char **argv )
{
// construct the viewer.
	osgViewer::Viewer viewer;


 // create the model
    osg::Group* root = new osg::Group;
    root->addChild( createShapes() );

    
    // add model to viewer.
    viewer.setSceneData( root );
osg::ref_ptr<osg::Camera> myCam = new osg::Camera;
myCam->setClearColor(backgroundColor); // black background
	// set dimensions of the view volume
	myCam->setProjectionMatrixAsPerspective(fovy, 3.0 / 3.0, 0.1, 3000);
	viewer.setCamera(myCam); // attach camera to the viewer
   
//return viewer.run();
	viewer.getCamera()->setUpdateCallback(new CameraUpdateCallback());

 
	viewer.setCameraManipulator(NULL);

	while(!viewer.done())
	{


		viewer.frame();
	}
	return 0;
}



//walkthrough of fly flying around inside black and white striped cylinder,black floor, navigating around inside cylinder (ie fps)