#define RENDER_SNIPPET 1

#ifdef RENDER_SNIPPET

#include <vector>
#include "PxPhysicsAPI.h"
#include "PxTkFPS.h"
#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
//#include <iostream>
//#include <ctime>

using namespace physx;

extern void initPhysics();
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern void createCoin(PxReal xAxis, PxReal yAxis, PxReal zAxis);
extern void pushByStep();
shdfnd::Time	mTimer;
PxReal delta;
PxReal lastTime;
float fps = 1.0f / 30.0f;
int now;

namespace
{
	Snippets::Camera* sCamera;
	PxToolkit::FPS mFPS;

	void motionCallback(int x, int y)
	{
		sCamera->handleMotion(x, y);
	}

	void keyboardCallback(unsigned char key, int x, int y)
	{
		if (key == 27)
			exit(0);

		if (!sCamera->handleKey(key, x, y))
			keyPress(key, sCamera->getTransform());
	}

	void mouseCallback(int button, int state, int x, int y)
	{
		sCamera->handleMouse(button, state, x, y);
	}

	void idleCallback()
	{

		delta = PxReal(mTimer.peekElapsedSeconds());
		if (delta - lastTime >= fps) {
			//std::cout << "time value: " << delta - lastTime << std::endl;
			lastTime = delta;
			glutPostRedisplay();
		}

		//glutPostRedisplay();
	}

	void renderCallback()
	{
		stepPhysics(true);

		Snippets::startRender(sCamera->getEye(), sCamera->getDir());

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
		}
		Snippets::finishRender();
	}

	void exitCallback(void)
	{
		delete sCamera;
		cleanupPhysics(true);
	}

}
void updateFPS()
{
	mFPS.update();
}

void displayFPS()
{
	char fpsText[512];
	sprintf(fpsText, "%0.2f fps", mFPS.getFPS());
	printf("%s\n", fpsText);

	//Renderer* renderer = getRenderer();

	//const PxU32 yInc = 18;
	//renderer->print(10, getCamera().getScreenHeight() - yInc * 2, fpsText);

}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(0, 200, 10), PxVec3(0, -1, -0.4));

	Snippets::setupDefaultWindow("PhysX Main");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);

	motionCallback(0, 0);
	atexit(exitCallback);
	initPhysics();
	glutMainLoop();
}

#endif
