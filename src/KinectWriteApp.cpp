#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ObjLoader.h"
#include "cinder/TriMesh.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/Fbo.h"
#include "cinder/params/Params.h"
#include "boost/date_time/posix_time/posix_time.hpp"

#include "Kinect.h"

using namespace ci;
using namespace ci::app;
using namespace std;


#define MAXFRAME 90

class KinectWriteApp : public AppBasic 
{
  public:
	void prepareSettings( ci::app::AppBasic::Settings *settings );
	void setup();
	void update();
	void draw();
	void keyDown( KeyEvent event );
	void shutdown();

	shared_ptr<thread>			mThread;

	bool						mStartWritingObjs;
	bool						mShouldQuit;

	ci::TriMesh					mMeshObj;

	void						writeObj();

	std::map<int, TriMesh>		mMeshWriter;

	int							mCurrentFrame;
	int							mFrameCounter;
	int							mStartFrame;

	bool						mStartWriting;

  private:
	KinectSdk::KinectRef		mKinect;
	vector<Vec3f>				mPoints;
	ci::CameraPersp				mCamera;
	float						mDepthThreaholdBack;
	float						mDepthThreaholdFront;

	params::InterfaceGlRef		mParams;
	
};

// Kinect image size
const Vec2i	kKinectSize( 640, 480 );

void KinectWriteApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate(30);
}

// Set up
void KinectWriteApp::setup()
{
	// Set up OpenGL
	gl::enable( GL_DEPTH_TEST );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
	glEnable( GL_POINT_SMOOTH );
	glPointSize( 0.25f );
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();

	// Start Kinect with isolated depth tracking only
	mKinect = KinectSdk::Kinect::create();
	mKinect->start( KinectSdk::DeviceOptions().enableSkeletonTracking( false ).enableColor( false ).setDepthResolution( KinectSdk::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 ) );

	// Set up camera
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 670.0f ), Vec3f::zero() );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 0.01f, 5000.0f );

	console() << "init thread" << std::endl;
	mStartWritingObjs = false;
	mShouldQuit = false;

	mStartWriting = false;

	mStartFrame	  = 0;
	mFrameCounter = 0;
	mCurrentFrame = 0;


	mDepthThreaholdBack = -0.1;
	mDepthThreaholdFront = 0.1;


	//params
	mParams = params::InterfaceGl::create("App parameters", Vec2i(250, 200));
	mParams->addParam("Depth Threshold Back", &mDepthThreaholdBack, "min = -1.5 max = 1.5 step=0.01");
	mParams->addParam("Depth Threshold Front", &mDepthThreaholdFront, "min = -1.5 max = 1.5 step=0.01");

	mThread = shared_ptr<thread>(new thread(bind(&KinectWriteApp::writeObj, this)));
}



void KinectWriteApp::update()
{
	// Device is capturing
	if (mKinect->isCapturing()) {
		mKinect->update();

		// Clear point list
		Vec3f offset(Vec2f(kKinectSize) * Vec2f(-0.5f, 0.5f));
		offset.z = mCamera.getEyePoint().z;
		Vec3f position = Vec3f::zero();
		mPoints.clear();

		// Iterate image rows
		for (int32_t y = 0; y < kKinectSize.y; y++) {
			for (int32_t x = 0; x < kKinectSize.x; x++) {
				// Read depth as 0.0 - 1.0 float
				float depth = mKinect->getDepthAt(Vec2i(x, y));

				// Add position to point list
				if (depth > mDepthThreaholdBack && depth < mDepthThreaholdFront){
					position.z = depth * mCamera.getEyePoint().z * -3.0f;
					mPoints.push_back(position * Vec3f(1.1f, -1.1f, 1.0f) + offset);
				}

				// Shift point
				position.x++;
			}

			// Update position
			position.x = 0.0f;
			position.y++;
		}
	}
	else {
		// If Kinect initialization failed, try again every 90 frames
		if (getElapsedFrames() % 90 == 0) {
			mKinect->start();
		}
	}

	
	mMeshObj.clear();
	if (mPoints.size() > 0){
		for (auto pointIt = mPoints.cbegin(); pointIt != mPoints.cend(); ++pointIt) {
			float depth = 1.0f - pointIt->z / mCamera.getEyePoint().z * -1.5f;
			//ci::Vec3f point = ci::Vec3f(pointIt->x / 320.0, pointIt->y / 240.0, pointIt->z / 1340.0);
			mMeshObj.appendVertex(*pointIt);
			mMeshObj.appendColorRgb(ColorAf(1.0f, depth, 1.0f - depth, depth));
		}
	}

	//mMeshObj.recalculateNormals();

	if (mStartWriting){
		if (mFrameCounter < MAXFRAME){
			mCurrentFrame = getElapsedFrames();
			mMeshWriter.insert(std::pair<int, ci::TriMesh>(mCurrentFrame, mMeshObj));
			console() << "added mesh to map, frame" << mCurrentFrame << " " << mFrameCounter << std::endl;
			mFrameCounter++;
		}
		else{
			console() << "START WRITING TO OBJS " << std::endl;
			mStartWriting = false;
			mStartWritingObjs = true;

		}
	}
}

void KinectWriteApp::draw()
{
	// Clear window
	gl::clear();
	gl::setMatrices( mCamera );

	//gl::enableDepthWrite();
	//gl::enableDepthRead();

	if (mPoints.size() > 0){
		gl::draw(gl::VboMesh(mMeshObj));
	}

	mParams->draw();
}

void KinectWriteApp::writeObj()
{
	ci::ThreadSetup threadSetup;
	
	while (!mShouldQuit){
		if (mStartWritingObjs){
			console() << "Start writing Objs" << std::endl;

			int i = 0;
			for (auto it = mMeshWriter.begin(); it != mMeshWriter.end(); ++it){
				int frame = it->first;
				ci::TriMesh triMesh = it->second;
				std::ostringstream oss;

				boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

				//oss << "../Objs/model_" << frame <<"_"<< now.date().day_number() << "_" << now.time_of_day().seconds() << ".obj";

				oss << "../Objs/model_" << frame << ".obj";
				console() << "start  " << oss.str() << " ... " << i << std::endl;
				try{
					ObjLoader::write(writeFile(oss.str()), triMesh);
				}
				catch (exception & e){
					console() << e.what() << std::endl;
				}
				console() << "Done writing Frame: " << frame << std::endl;
				i++;
			}
			console() << "Finalizing" << std::endl;
			mStartWritingObjs = false;
		}
	}
}

void KinectWriteApp::keyDown( KeyEvent event )
{
	// Key on key...
	switch( event.getCode() ) {
		case KeyEvent::KEY_ESCAPE:
			quit();
		break;
		case KeyEvent::KEY_f:
			setFullScreen( ! isFullScreen() );
		break;

		//save .obj
		case KeyEvent::KEY_1:
			console() << " start allocating Meshes" << std::endl;
			mStartWriting = true;
			break;
		case KeyEvent::KEY_0:
			mStartWritingObjs = true;
			break;
	}
}

void KinectWriteApp::shutdown()
{
	mShouldQuit = true;
	mKinect->stop();
	mPoints.clear();

	mThread->join();
}

CINDER_APP_BASIC( KinectWriteApp, RendererGl )
