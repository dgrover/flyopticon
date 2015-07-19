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

double screenHeight=1280;//1200;//768;
double screenWidth=800*4;//1920;//1366;
double xoffset = 1920;//0
double yoffset = 0;



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
    
    




	osg::ref_ptr<osg::Vec2Array> vertices = new osg::Vec2Array(4);
	(*vertices)[0].set(-screenWidth/2, -screenHeight/2); //left bottom
	(*vertices)[1].set(-screenWidth/2, screenHeight/2); //left top
	(*vertices)[2].set(screenWidth/2, -screenHeight/2); //right bottom 
	(*vertices)[3].set(screenWidth/2, screenHeight/2); //right top




	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLES, 6);
	(*indices) [0] = 0; (*indices) [1] = 1; (*indices) [2] = 2; //LB, LT, RB
	(*indices) [3] = 1; (*indices) [4] = 2; (*indices) [5] = 3; //LT, RB, RT



osg::ref_ptr<osg::Geometry> rect = new osg::Geometry;


float texWidth=1;
float texHeight=1;
	osg::Vec2Array* texcoords = new osg::Vec2Array(4);

(*texcoords)[0].set(0.0f, 0.0f);	//left bottom
(*texcoords)[1].set(0.0f, texHeight);	//left top
(*texcoords)[2].set(texWidth, 0.0f); //right bottom
(*texcoords)[3].set(texWidth, texHeight);	//right top



rect->setTexCoordArray(0, texcoords);




	

	rect->setVertexArray(vertices.get());
	rect->addPrimitiveSet(indices.get());

rect->setStateSet( stateset );
	geode->setCullingActive(false);




    geode->addDrawable(rect.get());
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
    root->addChild( createDisplay() );

    viewer.setSceneData( root );



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

       
       viewer.getCamera()->setGraphicsContext(gc.get());
       viewer.getCamera()->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        viewer.getCamera()->setDrawBuffer(buffer);
        viewer.getCamera()->setReadBuffer(buffer);
	

viewer.getCamera()->setProjectionMatrixAsOrtho2D(-screenWidth/2, screenWidth/2, -screenHeight/2, screenHeight/2);

viewer.getCamera()->setClearColor(backgroundColor); //  background



 
	viewer.setCameraManipulator(NULL);

	while(!viewer.done())
	{
		viewer.frame();
	}
	return 0;
}