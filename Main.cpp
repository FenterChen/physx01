#include "PxPhysics.h"
#include "PxPhysicsAPI.h"
#include <ctype.h>
#include "SnippetPrint.h"
#include "SnippetPVD.h"
#include "SnippetUtils.h"
#include <ctime>

#define RENDER_SNIPPET 

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;

PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

PxReal stackZ = 10.0f;
PxRigidDynamic* machineP;

bool direction = true;
float xLine = 0.0;
float stepFps = 30;
//int xLine = 0;


PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);

	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void createMachine()
{
	PxShape* machineShape = gPhysics->createShape(PxBoxGeometry(5, 10, 40), *gMaterial);
	PxShape* machineShapeB = gPhysics->createShape(PxBoxGeometry(55, 5, 40), *gMaterial);
	PxShape* machineShapeP = gPhysics->createShape(PxBoxGeometry(5, 5, 5), *gMaterial);
	PxShape* machineShapeP2 = gPhysics->createShape(PxBoxGeometry(2, 2, 2), *gMaterial);
	PxTransform machineTmL(PxVec3(PxReal(-50), PxReal(100), -40));
	PxTransform machineTmR(PxVec3(PxReal(50), PxReal(100), -40));
	PxTransform machineTmB(PxVec3(PxReal(0), PxReal(90), -40));
	PxTransform machineTmP(PxVec3(PxReal(0), PxReal(100), -40));
	PxTransform machineTmP2(PxVec3(PxReal(20), PxReal(100), -40));
	PxRigidStatic* machineL = gPhysics->createRigidStatic(machineTmL);
	PxRigidStatic* machineR = gPhysics->createRigidStatic(machineTmR);
	PxRigidStatic* machineB = gPhysics->createRigidStatic(machineTmB);
	machineP = gPhysics->createRigidDynamic(machineTmP);
	PxRigidDynamic* machineP2 = gPhysics->createRigidDynamic(machineTmP2);
	machineL->attachShape(*machineShape);
	machineR->attachShape(*machineShape);
	machineB->attachShape(*machineShapeB);
	machineP->attachShape(*machineShapeP);
	machineP2->attachShape(*machineShapeP2);
	PxRigidBodyExt::updateMassAndInertia(*machineP, 10.0f);
	gScene->addActor(*machineL);
	gScene->addActor(*machineR);
	gScene->addActor(*machineB);
	gScene->addActor(*machineP);
	gScene->addActor(*machineP2);
}

void push() {

	if (direction==true) {
		//machineP->setLinearVelocity(PxVec3(xLine = xLine + 1/stepFps, 0, 0));
		machineP->setGlobalPose(PxTransform(xLine= xLine++, 100, -40));
		if (xLine >= 20.0f) {
			direction = false;
		}
	}  else {
		//machineP->setLinearVelocity(PxVec3(xLine = xLine - 1/stepFps, 0, 0));
		machineP->setGlobalPose(PxTransform(xLine = xLine--, 100, -40));
		if (xLine <= -20.0f) {
			direction = true;
		}
	}
}

void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());

	//ENABLE_ENHANCED_DETERMINISM
	//sceneDesc.flags.set(PxSceneFlag::eENABLE_ENHANCED_DETERMINISM);

	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	//plane
	//PxRigidStatic* groundPlane = PxCrkeatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	//gScene->addActor(*groundPlane);

	//create Coin pusher machine
	createMachine();

	//for (PxU32 i = 0; i < 10; i++)
		//createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / stepFps);
	gScene->fetchResults(true);
	PxU32 timestamp = gScene->getTimestamp();
	extern void updateFPS();
	extern void displayFPS();
	updateFPS();
	displayFPS();
	if (timestamp % 30 == 0) {
		push();
	}


	//if (timestamp == 500) {
	//	createDynamic(PxTransform(PxVec3(0, 40, 10)), PxSphereGeometry(5), PxVec3(0, -50, -100));
	//}
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("PhysX Main done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	extern void displayFPS();
	switch (toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
	}
}

int snippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);

	//static const PxU32 frameCount = 100;
	//initPhysics(false);
	//for (PxU32 i = 0; i < frameCount; i++)
	//	stepPhysics(false);
	//cleanupPhysics(false);
#endif

	return 0;
}