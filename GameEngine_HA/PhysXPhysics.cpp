#include "PhysXPhysics.h"
#include <time.h>
//https://github.com/codetechandtutorials/PhysX-Setup/
void PhysXPhysics::Initialize()
{
	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mDefaultAllocatorCallback, mDefaultErrorCallback);
	if (!mFoundation) throw("PxCreateFoundation failed!");
	mPvd = PxCreatePvd(*mFoundation);
	mTransport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	mPvd->connect(*mTransport, physx::PxPvdInstrumentationFlag::eALL);
	mToleranceScale.length = 100;        // typical length of an object
	mToleranceScale.speed = 981;         // typical speed of an object, gravity*1s is a reasonable choice
	//mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(),true, mPvd);
	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, mToleranceScale, true, mPvd);
	physx::PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	mDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = mDispatcher;
	sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
	mScene = mPhysics->createScene(sceneDesc);

	physx::PxPvdSceneClient* pvdClient = mScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.6f);
}

void PhysXPhysics::ShutDown()
{
	mActors.clear();
	mScene->release();
	mDispatcher->release();
	mPvd->release();
	mTransport->release();
	mFoundation->release();
	mPhysics->release();
}

void PhysXPhysics::Update()
{
	mScene->simulate(1.0f / 60.0f);
	mScene->fetchResults(true);
}

void PhysXPhysics::Reset()
{
	// Loop through actors
	for (physx::PxRigidActor* actor : mActors)
	{
		physx::PxTransform originalTransform;
		physx::PxVec3 originalVelocity;
		originalTransform = mOriginalTransforms[actor];
		actor->setGlobalPose(originalTransform);
	}
}

void PhysXPhysics::createScene()
{
	// Create the ground
	float platformSize = 32.0f;
	physx::PxVec3 platformDimensions(platformSize, 1.0f, platformSize);
	physx::PxTransform platformTransform(physx::PxVec3(0.0f, -1.0f, 0.0f));
	physx::PxShape* platformShape = mPhysics->createShape(physx::PxBoxGeometry(platformDimensions), *mMaterial);
	physx::PxRigidStatic* platformActor = mPhysics->createRigidStatic(platformTransform);
	platformActor->attachShape(*platformShape);
	mScene->addActor(*platformActor);
	platformShape->release();
}

physx::PxRigidDynamic* PhysXPhysics::createSphere(float radius, glm::vec3 position, float mass)
{
	physx::PxTransform sphereTransform(physx::PxVec3(position.x, position.y, position.z));
	physx::PxShape* sphereShape = mPhysics->createShape(physx::PxSphereGeometry(radius), *mMaterial);
	physx::PxRigidDynamic* sphereActor = mPhysics->createRigidDynamic(sphereTransform);
	sphereActor->attachShape(*sphereShape);
	physx::PxRigidBodyExt::updateMassAndInertia(*sphereActor, mass);
	mActors.push_back(sphereActor);
	mScene->addActor(*sphereActor);
	mOriginalTransforms.emplace(sphereActor, sphereTransform);
	sphereShape->release();
	return sphereActor;
}

physx::PxRigidDynamic* PhysXPhysics::createRandomSphere(glm::vec2 radius, glm::vec3 minBounds, glm::vec3 maxBounds, glm::vec2 mass)
{
	// Seed rand
	std::srand(time(nullptr));

	// Random position between two bounds
	float x = (float)(std::rand() % (int)(maxBounds.x - minBounds.x + 1) + minBounds.x);
	float y = (float)(std::rand() % (int)(maxBounds.y - minBounds.y + 1) + minBounds.y);
	float z = (float)(std::rand() % (int)(maxBounds.z - minBounds.z + 1) + minBounds.z);
	physx::PxVec3 position(x, y, z);

	// Random radius and mass using the vec2.x and y as the bounds
	float r = (float)(std::rand() % (int)(radius.y - radius.x + 1) + radius.x);
	float m = (float)(std::rand() % (int)(mass.y - mass.x + 1) + mass.x);

	// Create Actor
	physx::PxTransform sphereTransform(physx::PxVec3(position.x, position.y, position.z));
	physx::PxShape* sphereShape = mPhysics->createShape(physx::PxSphereGeometry(r), *mMaterial);
	physx::PxRigidDynamic* sphereActor = mPhysics->createRigidDynamic(sphereTransform);
	sphereActor->attachShape(*sphereShape);
	physx::PxRigidBodyExt::updateMassAndInertia(*sphereActor, m);
	mActors.push_back(sphereActor);
	mScene->addActor(*sphereActor);
	sphereShape->release();
	return sphereActor;
}


physx::PxRigidDynamic* PhysXPhysics::createCube(float scale, glm::vec3 position, glm::quat rotation, float mass)
{
	physx::PxVec3 halfExtents(scale / 2.f, scale / 2.f, scale / 2.f);
	physx::PxTransform cubeTransform(physx::PxVec3(position.x, position.y, position.z), physx::PxQuat(rotation.x, rotation.y, rotation.z, rotation.w));
	physx::PxShape* cubeShape = mPhysics->createShape(physx::PxBoxGeometry(halfExtents), *mMaterial);
	physx::PxRigidDynamic* cubeActor = mPhysics->createRigidDynamic(cubeTransform);
	cubeActor->attachShape(*cubeShape);
	physx::PxRigidBodyExt::updateMassAndInertia(*cubeActor, mass);
	mActors.push_back(cubeActor);
	mOriginalTransforms.emplace(cubeActor, cubeTransform);
	mScene->addActor(*cubeActor);
	cubeShape->release();

	return cubeActor;
}

physx::PxRigidStatic* PhysXPhysics::createStaticCube(float scale, glm::vec3 position, glm::quat rotation)
{
	physx::PxVec3 halfExtents(scale / 2.f, scale / 2.f, scale / 2.f);
	physx::PxTransform transform(physx::PxVec3(position.x, position.y, position.z), physx::PxQuat(rotation.x, rotation.y, rotation.z, rotation.w));
	physx::PxShape* cubeShape = mPhysics->createShape(physx::PxBoxGeometry(halfExtents), *mMaterial);
	physx::PxRigidStatic* cubeActor = mPhysics->createRigidStatic(transform);
	cubeActor->attachShape(*cubeShape);
	mScene->addActor(*cubeActor);
	cubeShape->release();
	return cubeActor;
}

physx::PxRigidDynamic* PhysXPhysics::createCylinder(float radius, float halfHeight, glm::vec3 position, glm::quat rotation, float mass)
{
	physx::PxCapsuleGeometry cylinder(radius, halfHeight);
	physx::PxTransform cylinderTransform(physx::PxVec3(position.x, position.y, position.z), physx::PxQuat(rotation.x, rotation.y, rotation.z, rotation.w));
	physx::PxShape* cylinderShape = mPhysics->createShape(cylinder, *mMaterial);
	physx::PxRigidDynamic* cylinderActor = mPhysics->createRigidDynamic(cylinderTransform);
	cylinderActor->attachShape(*cylinderShape);
	physx::PxRigidBodyExt::updateMassAndInertia(*cylinderActor, mass);
	mActors.push_back(cylinderActor);
	mScene->addActor(*cylinderActor);
	mOriginalTransforms.emplace(cylinderActor, cylinderTransform);
	cylinderShape->release();
	return cylinderActor;
}