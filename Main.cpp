#include "PxPhysicsAPI.h"
#include <ctype.h>
#include "SnippetPVD.h"
#include "SnippetUtils.h"

#define RENDER_SNIPPET 

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;
PxCooking* mCooking = NULL;
PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;
PxMaterial* gMaterial = NULL;
PxMaterial* coinMaterial = NULL;
PxPvd* gPvd = NULL;
PxRigidDynamic* machineP;
PxReal stackZ = 10.0f;
//#include <iostream>
//#include <ctime>

bool direction = true;
float zLine = -80.0;
float stepFps = 1.0f / 50.0f;
int i = 0;
int lastTime = 0;

//creat coin
PxRigidDynamic* coinActor;
PxConvexMeshDesc coinConvexDesc;
const int numPoints = 32;
const float radius = 3.0f;
const float height = 2.0f;
PxVec3* coinConvexVerts = NULL;
PxQuat coinRotation(PxHalfPi, PxVec3(1, 0, 0));
PxConvexMesh* convexMesh;
PxShape* aConvexShape;

extern void updateFPS();
extern void displayFPS();


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
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), PxReal(0)) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void pushByStep() {
	if (direction == true) {
		//machineP->setLinearVelocity(PxVec3(xLine = xLine + 1/stepFps, 0, 0));
		//int elapsedTime = clock() / CLOCKS_PER_SEC;
		//machineP->setGlobalPose(PxTransform(0.0f, 95.0f, zLine = zLine - 0.2f));
		machineP->setKinematicTarget(PxTransform(0.0f, 95.0f, zLine = zLine - 0.2f));
		if (zLine <= -90.0f) {
			direction = false;
		}
	}
	else {
		//machineP->setLinearVelocity(PxVec3(xLine = xLine - 1/stepFps, 0, 0));
		machineP->setKinematicTarget(PxTransform(0.0f, 95.0f, zLine = zLine + 0.2f));
		if (zLine >= -65.0f) {
			direction = true;
		}
	}
}

void createCoin(PxReal xAxis, PxReal yAxis, PxReal zAxis) {
	coinConvexDesc.points.count = numPoints * 2;
	coinConvexDesc.points.stride = sizeof(PxVec3);

	if (coinConvexVerts == NULL) {
		coinConvexVerts = new PxVec3[coinConvexDesc.points.count];
		for (int i = 0; i < numPoints; i++) {
			float angle = (float)i / (float)numPoints * 2.0f * PxPi;
			coinConvexVerts[i] = PxVec3(PxReal(cos(angle) * radius), PxReal(sin(angle) * radius), PxReal(0));
			coinConvexVerts[i + numPoints] = PxVec3(PxReal(cos(angle) * radius), PxReal(sin(angle) * radius), PxReal(height));
		}
		coinConvexDesc.points.data = coinConvexVerts;
		coinConvexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
	}

	coinActor = gPhysics->createRigidDynamic(PxTransform(PxVec3(PxReal(xAxis), PxReal(yAxis), PxReal(zAxis)), coinRotation));
	coinMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.0f);

	PxDefaultMemoryOutputStream buf;
	PxConvexMeshCookingResult::Enum result;

	if (!mCooking->cookConvexMesh(coinConvexDesc, buf, &result)) {
		printf("cookConvexMesh failed");
	}
	else {
		PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
		convexMesh = gPhysics->createConvexMesh(input);
		aConvexShape = PxRigidActorExt::createExclusiveShape(*coinActor,
			PxConvexMeshGeometry(convexMesh), *coinMaterial);
		PxRigidBodyExt::updateMassAndInertia(*coinActor, 10.0f);
		gScene->addActor(*coinActor);
	}
}

//void createCoin() {
//	PxConvexMeshDesc convexDesc;
//	convexDesc.points.count = 5;
//	convexDesc.points.stride = sizeof(PxVec3);
//	static const PxVec3 convexVerts[] = { PxVec3(0,1,0),PxVec3(1,0,0),PxVec3(-1,0,0),PxVec3(0,0,1),
//	PxVec3(0,0,-1) };
//	convexDesc.points.data = convexVerts;
//	convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
//
//	PxRigidActor* coinActor = gPhysics->createRigidDynamic(PxTransform(PxVec3(PxReal(20), PxReal(100), PxReal(-40))));
//	mMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
//
//	PxDefaultMemoryOutputStream buf;
//	PxConvexMeshCookingResult::Enum result;
//
//	if (!mCooking->cookConvexMesh(convexDesc, buf, &result)) {
//		printf("cookConvexMesh failed");
//	}
//	else {
//		PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
//		PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);
//		PxShape* aConvexShape = PxRigidActorExt::createExclusiveShape(*coinActor,
//			PxConvexMeshGeometry(convexMesh), *mMaterial);
//		coinActor->attachShape(*aConvexShape);
//		gScene->addActor(*coinActor);
//	}
//
//}

void createMachine()
{
	PxShape* machineShape = gPhysics->createShape(PxBoxGeometry(5, 10, 40), *gMaterial);
	PxShape* machineShapeB = gPhysics->createShape(PxBoxGeometry(55, 5, 40), *gMaterial);
	PxShape* machineShapeP = gPhysics->createShape(PxBoxGeometry(45, 2, 20), *gMaterial);
	PxShape* machineShapeT = gPhysics->createShape(PxBoxGeometry(45, 6.5, 5), *gMaterial);

	PxTransform machineTmL(PxVec3(PxReal(-50), PxReal(100), PxReal(-40)));
	PxTransform machineTmR(PxVec3(PxReal(50), PxReal(100), PxReal(-40)));
	PxTransform machineTmB(PxVec3(PxReal(0), PxReal(90), PxReal(-40)));
	PxTransform machineTmT(PxVec3(PxReal(0), PxReal(103.5), PxReal(-75)));
	PxTransform machineTmP(PxVec3(PxReal(0), PxReal(95), PxReal(-80)));

	PxRigidStatic* machineL = gPhysics->createRigidStatic(machineTmL);
	PxRigidStatic* machineR = gPhysics->createRigidStatic(machineTmR);
	PxRigidStatic* machineB = gPhysics->createRigidStatic(machineTmB);
	PxRigidStatic* machineT = gPhysics->createRigidStatic(machineTmT);
	machineP = gPhysics->createRigidDynamic(machineTmP);
	machineP->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	machineL->attachShape(*machineShape);
	machineR->attachShape(*machineShape);
	machineB->attachShape(*machineShapeB);
	machineT->attachShape(*machineShapeT);
	machineP->attachShape(*machineShapeP);
	gScene->addActor(*machineL);
	gScene->addActor(*machineR);
	gScene->addActor(*machineB);
	gScene->addActor(*machineT);
	gScene->addActor(*machineP);

	machineShape->release();
	machineShapeB->release();
	machineShapeP->release();
	machineShapeT->release();
}

void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(gPhysics->getTolerancesScale()));
	if (!mCooking)
		printf("PxCreateCooking failed!");

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());

	//ENABLE_ENHANCED_DETERMINISM
	sceneDesc.flags.set(PxSceneFlag::eENABLE_ENHANCED_DETERMINISM);

	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(8);
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
	//PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	//gScene->addActor(*groundPlane);

	//create Coin pusher machine
	createMachine();


	//for (PxU32 i = 0; i < 10; i++)
		//createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

}

void stepPhysics(bool /*interactive*/)
{
	//std::cout << "Delta value: " << timestamp << std::endl;
	gScene->simulate(stepFps);
	gScene->fetchResults(true);
	PxU32 timestamp = gScene->getTimestamp();
	//timestamp = gScene->getTimestamp();
	//std::cout << "Delta value: " << timestamp << std::endl;
	updateFPS();
	if (timestamp % 50 == 0) {
		displayFPS();
	}
	//if (timestamp % 2 == 0) {
		pushByStep();
	//}
	//if (now - lastTime > 0) {
	//	pushByStep();
	//	lastTime = now;
	//}
	// 
	//std::cout << "Delta value: " << Delta << std::endl;
	//if (((int)Delta) % 1 == 0) {
	//	/*std::cout << "Delta value: " << (int)Delta << std::endl;*/
	//	push();
	//}
	if (timestamp == 120) {
		createCoin(0, 105, -65);
		createCoin(-20, 105, -65);
		createCoin(20, 105, -65);
		createCoin(30, 105, -65);
		createCoin(-30, 105, -65);
	}
	if (timestamp % (50*5) == 0) {
		createCoin(0, 105, -65);
	}	
	if (timestamp % (50*8) == 0) {
		createCoin(-20, 105, -65);
	}	
	if (timestamp % (50*12) == 0) {
		createCoin(20, 105, -65);
	}	
	if (timestamp % (50*16) == 0) {
		createCoin(30, 105, -65);
	}	
	if (timestamp % (50*20) == 0) {
		createCoin(-30, 105, -65);
	}

	//create Coin init postion
	if (timestamp == 30) {
		createCoin(-40, 97, -40);
		createCoin(-30, 97, -40);
		createCoin(-20, 97, -40);
		createCoin(-10, 97, -40);
		createCoin(0, 97, -40);
		createCoin(10, 97, -40);
		createCoin(20, 97, -40);
		createCoin(30, 97, -40);
		createCoin(40, 97, -40);
	}

	if (timestamp == 60) {
		createCoin(-40, 97, -30);
		createCoin(-30, 97, -30);
		createCoin(-20, 97, -30);
		createCoin(-10, 97, -30);
		createCoin(0, 97, -30);
		createCoin(10, 97, -30);
		createCoin(20, 97, -30);
		createCoin(30, 97, -30);
		createCoin(40, 97, -30);
	}

	if (timestamp == 90) {
		createCoin(-40, 97, -20);
		createCoin(-30, 97, -20);
		createCoin(-20, 97, -20);
		createCoin(-10, 97, -20);
		createCoin(0, 97, -20);
		createCoin(10, 97, -20);
		createCoin(20, 97, -20);
		createCoin(30, 97, -20);
		createCoin(40, 97, -20);
	}

	if (timestamp == 120) {
		createCoin(-40, 97, -10);
		createCoin(-30, 97, -10);
		createCoin(-20, 97, -10);
		createCoin(-10, 97, -10);
		createCoin(0, 97, -10);
		createCoin(10, 97, -10);
		createCoin(20, 97, -10);
		createCoin(30, 97, -10);
		createCoin(40, 97, -10);
	}



	//if (timestamp == 500) {
	//	createDynamic(PxTransform(PxVec3(0, 40, 10)), PxSphereGeometry(5), PxVec3(0, -50, -100));
	//}
}

void cleanupPhysics(bool /*interactive*/)
{
	delete coinConvexVerts;
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(mCooking);
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