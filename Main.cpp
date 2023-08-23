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


PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxReal addTime = 10.0f;
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

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());

	//ENABLE_ENHANCED_DETERMINISM
	sceneDesc.flags.set(PxSceneFlag::eENABLE_ENHANCED_DETERMINISM);

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

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	for (PxU32 i = 0; i < 10; i++)
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);


	if (!interactive)
		createDynamic(PxTransform(PxVec3(0, 40, 50)), PxSphereGeometry(10), PxVec3(0, -50, -100));
}

void stepPhysics(bool /*interactive*/)
{
	extern void updateFPS();
	extern void displayFPS();
	updateFPS();
	displayFPS();
	gScene->simulate(1.0f / 30.0f);
	gScene->fetchResults(true);
	PxU32 timestamp = gScene->getTimestamp();
	if (timestamp == 30) {
		createDynamic(PxTransform(PxVec3(0, 40, 50)), PxSphereGeometry(5), PxVec3(0, -50, -100));
	}
	if (timestamp == 150) {
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 20.0f)), 10, 2.0f);
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 20.0f)), 10, 2.0f);
		createDynamic(PxTransform(PxVec3(0, 40, 40)), PxSphereGeometry(10), PxVec3(0, -50, -100));
	}
	if (timestamp == 300) {
		createDynamic(PxTransform(PxVec3(0, 40, 50)), PxSphereGeometry(5), PxVec3(0, -50, -100));
	}
	if (timestamp == 400) {
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 20.0f)), 10, 2.0f);
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 20.0f)), 10, 2.0f);
		createDynamic(PxTransform(PxVec3(0, 40, 10)), PxSphereGeometry(10), PxVec3(0, -50, -100));
	}
	if (timestamp == 500) {
		createDynamic(PxTransform(PxVec3(0, 40, 10)), PxSphereGeometry(5), PxVec3(0, -50, -100));
	}
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