#include <osg/Notify>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/TexMat>
#include <osg/Group>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>
#include <osg/Texture2D>
#include <osg/Math>

double screenHeight=1280;//768;
double screenWidth=800*4;//1366;
double xoffset = 1920;//0
double yoffset = 0;

double boxWidth=screenWidth/4;
double boxHeight = screenHeight;


osg::Vec4 backgroundColor = osg::Vec4(0, 1, 0, 1);

double depth=0;

double camHorLoc=0;
double camVertLoc=0;
double distance=1;
osg::Vec3d up=osg::Vec3d(0,0,1); //up vector

//std::string imageFileName="color2.jpg";
std::string imageFileName="numberline.gif";

double transRate=0.3;//0.3;








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
            return;

        if (nv->getFrameStamp())
		{
            double currTime = nv->getFrameStamp()->getSimulationTime();
				float s=currTime*transRate;

				texMat->setMatrix(osg::Matrix::translate(s,0.0,0.0));
        }
    }
};








osg::Geode* createWalls()
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
    
    




	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(10);
	(*vertices)[0].set(-boxWidth/2, -boxWidth/2, -boxHeight/2); //left front bottom
	(*vertices)[1].set(boxWidth/2, -boxWidth/2, -boxHeight/2); //right front bottom
	(*vertices)[2].set(boxWidth/2, -boxWidth/2, boxHeight/2); //right front top
	(*vertices)[3].set(-boxWidth/2, -boxWidth/2, boxHeight/2); //left front top
	(*vertices)[4].set(-boxWidth/2, boxWidth/2, -boxHeight/2); //left back bottom
	(*vertices)[5].set(boxWidth/2, boxWidth/2, -boxHeight/2); //right back bottom
	(*vertices)[6].set(boxWidth/2, boxWidth/2, boxHeight/2); //right back top
	(*vertices)[7].set(-boxWidth/2, boxWidth/2, boxHeight/2); //left back top
	(*vertices)[8].set(-boxWidth/2, -boxWidth/2, -boxHeight/2); //left front bottom
	(*vertices)[9].set(-boxWidth/2, -boxWidth/2, boxHeight/2); //left front top




	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLES, 24);
	(*indices) [0] = 0; (*indices) [1] = 1; (*indices) [2] = 2; //front (LFB, RFB, RFT)
	(*indices) [3] = 0; (*indices) [4] = 2; (*indices) [5] = 3; //front	(LFB, RFT, LFT)
	(*indices) [6] = 8; (*indices) [7] = 9; (*indices) [8] = 7; //left	(LFB, LFT, LBT)
	(*indices) [9] = 8; (*indices) [10] = 7; (*indices) [11] = 4; //left	(LFB, LBT, LBB)
	(*indices) [12] = 4; (*indices) [13] = 7; (*indices) [14] = 6; //back	(LBB, LBT, RBT)
	(*indices) [15] = 4; (*indices) [16] = 6; (*indices) [17] = 5; //back	(LBB, RBT, RBB)
	(*indices) [18] = 5; (*indices) [19] = 6; (*indices) [20] = 2; //right	(RBB, RBT, RFT)
	(*indices) [21] = 5; (*indices) [22] = 2; (*indices) [23] = 1;  //right	(RBB, RFT, RFB)


osg::ref_ptr<osg::Geometry> cube = new osg::Geometry;


float texWidth=1;
float texHeight=1;
	osg::Vec2Array* texcoords = new osg::Vec2Array(10);

(*texcoords)[0].set(0.0f, 0.0f);	//left front bottom of box
(*texcoords)[1].set(texWidth/4, 0.0f);	//right front bottom
(*texcoords)[2].set(texWidth/4, texHeight); //right front top
(*texcoords)[3].set(0.0f, texHeight);	//left front top
(*texcoords)[4].set(3*texWidth/4, 0.0f);	//left back bottom
(*texcoords)[5].set(texWidth/2, 0.0f);	//right back bottom
(*texcoords)[6].set(texWidth/2, texHeight);	//right back top
(*texcoords)[7].set(3*texWidth/4, texHeight);	//left back top
(*texcoords)[8].set(texWidth, 0);	//left front bottom
(*texcoords)[9].set(texWidth, texHeight);	//left front top


cube->setTexCoordArray(0, texcoords);




	

	cube->setVertexArray(vertices.get());
	cube->addPrimitiveSet(indices.get());

cube->setStateSet( stateset );
	geode->setCullingActive(false);




    geode->addDrawable(cube.get());
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
    root->addChild( createWalls() );

    viewer.setSceneData( root );



bool doubleBuf=true;
bool border=false;

	int width=screenWidth/4;
	int height=screenHeight;
	

    // front
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset + width*2;
        traits->y = yoffset;
        traits->width = width;
        traits->height = height;
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
	
		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
    }
    
    // back
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset + 0;
        traits->y = yoffset;
        traits->width = width;
        traits->height = height;
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


	viewer.addSlave(camera.get(), osg::Matrixd(),camTrans*camRot);
    }


//left side
	
 {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset+width;
        traits->y = yoffset;
        traits->width = width;
        traits->height = height;
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
osg::Matrixd camTrans1;
osg::Matrixd camTrans2;
camTrans1.makeTranslate(0.0,0.0,distance);
camRot.makeRotate(osg::DegreesToRadians(90.0), osg::Vec3(0,1,0));
camTrans2.makeTranslate(0.0,0.0,-1*distance);

	viewer.addSlave(camera.get(), osg::Matrixd(),camTrans1*camRot*camTrans2);;
    }








//right side
	
 {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = xoffset+3*width;
        traits->y = yoffset;
        traits->width = width;
        traits->height = height;
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
osg::Matrixd camTrans1;
osg::Matrixd camTrans2;
camTrans1.makeTranslate(0.0,0.0,distance);
camRot.makeRotate(osg::DegreesToRadians(-90.0), osg::Vec3(0,1,0));
camTrans2.makeTranslate(0.0,0.0,-1*distance);

//viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::translate(0.0,0.0,distance*2));
	viewer.addSlave(camera.get(), osg::Matrixd(),camTrans1*camRot*camTrans2);
	//viewer.addSlave(camera.get(), osg::Matrixd(),osg::Matrixd::rotate(osg::DegreesToRadians(180.0), osg::Vec3(0,1,0)).translate(0.0,0.0,distance*2));
    }





//viewer.getCamera()->setProjectionMatrixAsPerspective(28, boxWidth/boxHeight, 0.1, 100);
viewer.getCamera()->setProjectionMatrixAsOrtho2D(-boxWidth/2, boxWidth/2, -boxHeight/2, boxHeight/2);

viewer.getCamera()->setClearColor(backgroundColor); // black background

viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up); //eyepos, center, up


 
	viewer.setCameraManipulator(NULL);

	while(!viewer.done())
	{
		viewer.frame();
	}
	return 0;
}