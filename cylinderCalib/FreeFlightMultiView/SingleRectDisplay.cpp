#include <osg/TexMat>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Texture2D>


double screenHeight=768;//1280;
double screenWidth=1366;//800*4;
double xoffset = 0;//1920;
double yoffset = 0;
double boxWidth=screenWidth/4;
double boxHeight = screenHeight;
double depth=0;
double camHorLoc=0;
double camVertLoc=0;
double distance=1;

osg::Vec3d up=osg::Vec3d(0,0,1); //up vector
bool doubleBuf=true;
bool border=false;
std::string imageFileName="numberline.gif";
double transRate=0.3;



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
		texture->setUnRefImageDataAfterApply(true);
	}




	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
	(*vertices)[0].set(-boxWidth/2, boxWidth/2, -boxHeight/2);
	(*vertices)[1].set(boxWidth/2, boxWidth/2, -boxHeight/2);
	(*vertices)[2].set(boxWidth/2, boxWidth/2, boxHeight/2);
	(*vertices)[3].set(-boxWidth/2, boxWidth/2, boxHeight/2);



	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLES, 6);
	(*indices) [0] = 0; (*indices) [1] = 3; (*indices) [2] = 2; 
	(*indices) [3] = 0; (*indices) [4] = 2; (*indices) [5] = 1; 

	osg::ref_ptr<osg::Geometry> cube = new osg::Geometry;


	float texWidth=1;
	float texHeight=1;
	osg::Vec2Array* texcoords = new osg::Vec2Array(4);

	(*texcoords)[0].set(0, 0.0f);
	(*texcoords)[1].set(texWidth, 0.0f);
	(*texcoords)[2].set(texWidth, texHeight);
	(*texcoords)[3].set(0, texHeight);

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

int main(int argc, char **argv)
{
	osgViewer::Viewer viewer;
	osg::Group* root = new osg::Group;
	root->addChild(createWalls());
	viewer.setSceneData(root);


	int width=screenWidth;
	int height=screenHeight;

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = xoffset;
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
	viewer.getCamera()->setProjectionMatrixAsOrtho2D(-boxWidth/2, boxWidth/2, -boxHeight/2, boxHeight/2);
	viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(camHorLoc,-distance,camVertLoc), osg::Vec3d(camHorLoc,depth,camVertLoc), up); //eyepos, center, up



	while(!viewer.done())
	{
		viewer.frame();
	}
	return 0;
}