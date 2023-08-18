#include "PxPhysics.h"
#include "PxPhysicsAPI.h"

int main() {
	// declare variables
	physx::PxDefaultAllocator      mDefaultAllocatorCallback;
	physx::PxDefaultErrorCallback  mDefaultErrorCallback;
	physx::PxDefaultCpuDispatcher* mDispatcher = NULL;
	physx::PxTolerancesScale       mToleranceScale;

	physx::PxFoundation* mFoundation = NULL;
	physx::PxPhysics* mPhysics = NULL;

	physx::PxScene* mScene = NULL;
	physx::PxMaterial* mMaterial = NULL;

	physx::PxPvd* mPvd = NULL;
}