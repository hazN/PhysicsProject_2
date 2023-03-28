#include "PhysicsWorld.h"

#include <iostream>

#include <algorithm>
#include <math.h>

//#include <pvd/PxPvd.h>
//#include <extensions/PxSimpleFactory.h>

namespace physics
{
	PhysicsWorld::PhysicsWorld(void)
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

	PhysicsWorld::~PhysicsWorld(void)
	{
		mActors.clear();
		mScene->release();
		mDispatcher->release();
		mPvd->release();
		mTransport->release();
		mFoundation->release();
		mPhysics->release();
	}

	void PhysicsWorld::TimeStep(float dt)
	{
		mScene->simulate(1.0f / 60.0f);
		mScene->fetchResults(true);
	}

	void PhysicsWorld::SetGravity(const Vector3& gravity)
	{

	}

	void PhysicsWorld::AddBody(iCollisionBody* body)
	{

	}

	void PhysicsWorld::RemoveBody(iCollisionBody* body)
	{
		mActors[(iRigidBody*)body]->release();
		mActors.erase((iRigidBody*)body);
	}
};
