#include <osg/TexMat>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osgGA/GUIEventHandler>
#include <stdio.h>

double imageWidth=800*4;//1920;//5120;
double imageHeight=1280;//1200;//800;
int xoffset =1920;
int yoffset = 0;
int width=imageWidth/4;
int height=imageHeight;

double camHorLoc=0;
double camVertLoc=0;

float cradius = (7.5/2.0);// * 1280.0/800.0; ///7.5 diameter
float cheight = 8.0;// * 800.0/1280.0;//3.6; //8 height
double distance=cradius + 10.5;//14;//;


int slaveNum=0;



float transInc=0.1;
float rotInc=1;


osg::Matrixd cam1StartingView;
osg::Matrixd cam2StartingView;
osg::Matrixd cam3StartingView;
osg::Matrixd cam4StartingView;


osg::Matrixd cam1DefaultView=osg::Matrixd();
osg::Matrixd cam2DefaultView=osg::Matrixd::translate(0.0, 0.0, distance)*osg::Matrixd::rotate(osg::DegreesToRadians(-90.0), osg::Vec3(0,1,0))*osg::Matrixd::translate(0.0, 0.0, -1*distance);
osg::Matrixd cam3DefaultView=osg::Matrixd::translate(0.0, 0.0, distance*2)*osg::Matrixd::rotate(osg::DegreesToRadians(180.0), osg::Vec3(0,1,0));
osg::Matrixd cam4DefaultView=osg::Matrixd::translate(0.0, 0.0, distance)*osg::Matrixd::rotate(osg::DegreesToRadians(90.0), osg::Vec3(0,1,0))*osg::Matrixd::translate(0.0, 0.0, -1*distance);


osg::Matrixd cam1View;
osg::Matrixd cam2View;
osg::Matrixd cam3View;
osg::Matrixd cam4View;


osg::Matrixd transZforward = osg::Matrixd::translate(0.0,0.0,transInc);
osg::Matrixd transZbackward = osg::Matrixd::translate(0.0,0.0,transInc*-1);
osg::Matrixd transXleft = osg::Matrixd::translate(transInc,0.0,0.0); //trans cam left
osg::Matrixd transXright = osg::Matrixd::translate(transInc*-1,0.0,0.0);
osg::Matrixd rotcw = osg::Matrixd::rotate(osg::DegreesToRadians(rotInc), osg::Vec3(0,1,0)); //rotate camera clockwise
osg::Matrixd rotccw = osg::Matrixd::rotate(osg::DegreesToRadians(rotInc*-1), osg::Vec3(0,1,0));



osg::Vec4 backgroundColor = osg::Vec4(0, 0, 0, 1);

double diam=3.14159*2;
double depth=0;

double rS= 0.1; //rotation speed

osg::Vec3d up=osg::Vec3d(0,0,1); //up vector
const char* imageFileName="images//numberline.gif";//"vert_stripe.bmp";
const char* displayFile="displaySettings.txt";


osgViewer::Viewer viewer;


void printInfo(int cam);
void setStartingViews();


class keyboardHandler : public osgGA::GUIEventHandler
{
public:
	virtual bool handle(const osgGA::GUIEventAdapter&,osgGA::GUIActionAdapter&);
};

bool keyboardHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
	switch(ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			switch(ea.getKey())
			{
			case '0':
				slaveNum=0;
				break;
			case '1':
				slaveNum=1;
				break;
			case '2':
				slaveNum=2;
				break;
			case '3':
				slaveNum=3;
				break;
			case '4':
				slaveNum=4;
				break;

			case 'a':
			case 'A':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1View*transXleft;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2View*transXleft;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3View*transXleft;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4View*transXleft;
				}
				break;

			case 'd':
			case 'D':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1View*transXright;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2View*transXright;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3View*transXright;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4View*transXright;
				}

				break;

			case 'w':
			case 'W':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1View*transZforward;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2View*transZforward;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3View*transZforward;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4View*transZforward;
				}

				break;

			case 's':
			case 'S':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1View*transZbackward;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2View*transZbackward;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3View*transZbackward;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4View*transZbackward;
				}
				break;

			case 'q':
			case 'Q':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1View*rotccw;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2View*rotccw;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3View*rotccw;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4View*rotccw;
				}
				break;


			case 'e':
			case 'E':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1View*rotcw;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2View*rotcw;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3View*rotcw;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4View*rotcw;
				}
				break;

			case ' ':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1StartingView;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2StartingView;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3StartingView;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4StartingView;
				}
				break;

			case '.':
				if(slaveNum==0 || slaveNum==1)
				{
					cam1View=cam1DefaultView;
				}
				if(slaveNum==0 || slaveNum==2)
				{
					cam2View=cam2DefaultView;
				}
				if(slaveNum==0 || slaveNum==3)
				{
					cam3View=cam3DefaultView;
				}
				if(slaveNum==0 || slaveNum==4)
				{
					cam4View=cam4DefaultView;
				}
				break;

			case '+':
				transInc=transInc*2.0;
				rotInc=rotInc*2.0;
				transZforward = osg::Matrixd::translate(0.0,0.0,transInc);
				transZbackward = osg::Matrixd::translate(0.0,0.0,transInc*-1);
				transXleft = osg::Matrixd::translate(transInc,0.0,0.0); //trans cam left
				transXright = osg::Matrixd::translate(transInc*-1,0.0,0.0);
				rotcw = osg::Matrixd::rotate(osg::DegreesToRadians(rotInc), osg::Vec3(0,1,0)); //rotate camera clockwise
				rotccw = osg::Matrixd::rotate(osg::DegreesToRadians(rotInc*-1), osg::Vec3(0,1,0));
				break;

			case '-':
				transInc=transInc/2.0;
				rotInc=rotInc/2.0;
				transZforward = osg::Matrixd::translate(0.0,0.0,transInc);
				transZbackward = osg::Matrixd::translate(0.0,0.0,transInc*-1);
				transXleft = osg::Matrixd::translate(transInc,0.0,0.0); //trans cam left
				transXright = osg::Matrixd::translate(transInc*-1,0.0,0.0);
				rotcw = osg::Matrixd::rotate(osg::DegreesToRadians(rotInc), osg::Vec3(0,1,0)); //rotate camera clockwise
				rotccw = osg::Matrixd::rotate(osg::DegreesToRadians(rotInc*-1), osg::Vec3(0,1,0));
				break;

			case 'p':
			case 'P':
				printInfo(slaveNum);
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
		osg::Quat* quat = new osg::Quat(rS*diam*currTime, *rotationVec);


		osg::Geode * geode = dynamic_cast< osg::Geode * >( node ); 
		osg::Drawable* drawable = geode->getDrawable(0);
		//osg::ShapeDrawable * geom = dynamic_cast< osg::ShapeDrawable * >( node ) ; 

		osg::Shape* cyl = ((osg::ShapeDrawable*) drawable)->getShape();
		((osg::Cylinder*)cyl)->setRotation(*quat);
		((osg::ShapeDrawable*) drawable)->setShape(cyl);


	} 
}; 

/*
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
*/




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


	osg::TessellationHints* hints = new osg::TessellationHints;
	hints->setDetailRatio(0.8f);


	hints->setCreateBackFace(true);
	hints->setCreateFrontFace(false);
	hints->setCreateNormals(false);
	hints->setCreateTop(false);
	hints->setCreateBottom(false);
	osg::Vec3f* rotationVec=new osg::Vec3(0,0,1);
	osg::Quat* quat = new osg::Quat(180, *rotationVec);
	osg::Cylinder* cyl = new osg::Cylinder(osg::Vec3(0.0f,0.0f,0.0f),cradius,cheight);
	cyl->setRotation(*quat);
	osg::ShapeDrawable* cylinder = new osg::ShapeDrawable(cyl,hints);
	cylinder->setUseDisplayList(false);
	geode->addDrawable(cylinder);

	//geode->setUpdateCallback(new GeodeUpdateCallback());

	//osg::TexMat* texmat = (osg::TexMat*)(stateset->getTextureAttribute(0,osg::StateAttribute::TEXMAT));	
	//geode->setUpdateCallback(new TextureUpdateCallback(texmat));
	return geode;
}

void printInfo(int cam)
{
	osg::Quat::value_type  angle=* new osg::Quat::value_type();
	osg::Quat::value_type  rotVecX=* new osg::Quat::value_type();
	osg::Quat::value_type  rotVecY=* new osg::Quat::value_type();
	osg::Quat::value_type  rotVecZ=* new osg::Quat::value_type();
	printf("**********\n");

	for(int i=0; i<4; i++)
	{
		if(cam==0 || cam==i+1)
		{
			printf("Camera %i X Position: %f\n", i+1, viewer.getSlave(i)._viewOffset.getTrans().x());
			printf("Camera %i Z Position: %f\n", i+1, viewer.getSlave(i)._viewOffset.getTrans().z());
			viewer.getSlave(i)._viewOffset.getRotate().getRotate(angle, rotVecX, rotVecY, rotVecZ);
			printf("Camera %i Rotation: %f\n", i+1, osg::RadiansToDegrees(angle));
			printf("Camera %i Rotation Vector X: %f\n", i+1, rotVecX);
			printf("Camera %i Rotation Vector Y: %f\n", i+1, rotVecY);
			printf("Camera %i Rotation Vector Z: %f\n", i+1, rotVecZ);
			printf("\n");
		}
	}
	printf("**********\n\n");
}


void writeInfo()
{
	osg::Quat::value_type  angle=* new osg::Quat::value_type();
	osg::Quat::value_type  rotVecX=* new osg::Quat::value_type();
	osg::Quat::value_type  rotVecY=* new osg::Quat::value_type();
	osg::Quat::value_type  rotVecZ=* new osg::Quat::value_type();
	FILE * file= fopen(displayFile, "w");

	for(int i=0; i<4; i++)
	{
		fprintf(file, "%f\n", viewer.getSlave(i)._viewOffset.getTrans().x());
		fprintf(file, "%f\n", viewer.getSlave(i)._viewOffset.getTrans().z());
		viewer.getSlave(i)._viewOffset.getRotate().getRotate(angle, rotVecX, rotVecY, rotVecZ);
		fprintf(file, "%f\n", osg::RadiansToDegrees(angle));
		fprintf(file, "%f\n", rotVecX);
		fprintf(file, "%f\n", rotVecY);
		fprintf(file, "%f\n", rotVecZ);
	}

	fclose(file);
}

void setStartingViews()
{
	float number;
	std::vector<float> numbers;
	std::ifstream file(displayFile, std::ios::in);

	if(file.is_open())
	{
		while (file >> number) 
		{
			numbers.push_back(number);	
		}

		file.close();

		osg::Matrixd m1, m2, m3, m4;
		m1.setTrans(numbers[0], 0.0, numbers[1]);
		m1.setRotate(osg::Quat(osg::DegreesToRadians(numbers[2]), osg::Vec3d(numbers[3], numbers[4], numbers[5]))); 
		m2.setTrans(numbers[6], 0.0, numbers[7]);
		m2.setRotate(osg::Quat(osg::DegreesToRadians(numbers[8]), osg::Vec3d(numbers[9], numbers[10], numbers[11]))); 
		m3.setTrans(numbers[12], 0.0, numbers[13]);
		m3.setRotate(osg::Quat(osg::DegreesToRadians(numbers[14]), osg::Vec3d(numbers[15], numbers[16], numbers[17]))); 
		m4.setTrans(numbers[18], 0.0, numbers[19]);
		m4.setRotate(osg::Quat(osg::DegreesToRadians(numbers[20]), osg::Vec3d(numbers[21], numbers[22], numbers[23]))); 

		cam1StartingView=m1;
		cam2StartingView=m2;
		cam3StartingView=m3;
		cam4StartingView=m4;

		//cam1StartingView=osg::Matrixd::translate(numbers[0], 0.0, -1*numbers[1])*osg::Matrixd::rotate(osg::DegreesToRadians(numbers[2]), osg::Vec3d(numbers[3],numbers[4],numbers[5]));
		//cam2StartingView=osg::Matrixd::translate(-1*numbers[6], 0.0, numbers[7])*osg::Matrixd::rotate(osg::DegreesToRadians(numbers[8]), osg::Vec3d(numbers[9],numbers[10],numbers[11]));
		//cam3StartingView=osg::Matrixd::translate(numbers[12], 0.0, -1*numbers[13])*osg::Matrixd::rotate(osg::DegreesToRadians(numbers[14]), osg::Vec3d(numbers[15],numbers[16],numbers[17]));
		//cam4StartingView=osg::Matrixd::translate(-1*numbers[18], 0.0, numbers[19])*osg::Matrixd::rotate(osg::DegreesToRadians(numbers[20]), osg::Vec3d(numbers[21],numbers[22],numbers[23]));
	}
	else
	{
		cam1StartingView=cam1DefaultView;
		cam2StartingView=cam2DefaultView;
		cam3StartingView=cam3DefaultView;
		cam4StartingView=cam4DefaultView;
	}
}



int main( int argc, char **argv )
{

	osg::Group* root = new osg::Group;
	root->addChild( createShapes() );


	viewer.setSceneData( root );

	setStartingViews();

	cam1View=cam1StartingView;
	cam2View=cam2StartingView;
	cam3View=cam3StartingView;
	cam4View=cam4StartingView;


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
		viewer.addSlave(camera.get(), osg::Matrixd(),cam2View);	
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
		viewer.addSlave(camera.get(), osg::Matrixd(),cam3View);
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

		viewer.addSlave(camera.get(), osg::Matrixd(),cam4View);
	}




	keyboardHandler* handler = new keyboardHandler();
	viewer.addEventHandler(handler); 
	viewer.getCamera()->setClearColor(backgroundColor); // black background
	viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up);
	viewer.getCamera()->setProjectionMatrixAsPerspective(40.0, ((double)width)/((double)height), 0.1, 5);
	viewer.setCameraManipulator(NULL);

	while(!viewer.done())
	{
		viewer.getSlave(0)._viewOffset=cam1View;
		viewer.getSlave(1)._viewOffset=cam2View;
		viewer.getSlave(2)._viewOffset=cam3View;
		viewer.getSlave(3)._viewOffset=cam4View;

		viewer.frame();
	}

	printInfo(0);
	writeInfo();
	return 0;
}