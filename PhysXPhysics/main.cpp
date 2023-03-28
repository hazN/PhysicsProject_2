#include <physx/PxPhysicsAPI.h>

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


    // init physx
    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mDefaultAllocatorCallback, mDefaultErrorCallback);
    if (!mFoundation) throw("PxCreateFoundation failed!");
    mPvd = PxCreatePvd(*mFoundation);
    physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    mPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
    //mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(),true, mPvd);
    mToleranceScale.length = 100;        // typical length of an object
    mToleranceScale.speed = 981;         // typical speed of an object, gravity*1s is a reasonable choice
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, mToleranceScale, true, mPvd);
    //mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, mToleranceScale);

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


    // create simulation
    mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    // create balls
    float ballRadius = 0.5f;
    physx::PxTransform ball1Transform(physx::PxVec3(0.0f, 1.0f, 0.0f));
    physx::PxTransform ball2Transform(physx::PxVec3(1.0f, 1.0f, 0.0f));
    physx::PxShape* ballShape = mPhysics->createShape(physx::PxSphereGeometry(ballRadius), *mMaterial);
    physx::PxRigidDynamic* ball1Actor = mPhysics->createRigidDynamic(ball1Transform);
    physx::PxRigidDynamic* ball2Actor = mPhysics->createRigidDynamic(ball2Transform);
    ball1Actor->attachShape(*ballShape);
    ball2Actor->attachShape(*ballShape);
    physx::PxRigidBodyExt::updateMassAndInertia(*ball1Actor, 10.0f);
    physx::PxRigidBodyExt::updateMassAndInertia(*ball2Actor, 10.0f);
    mScene->addActor(*ball1Actor);
    mScene->addActor(*ball2Actor);

    ballShape->release();

    // run simulation
    while (1) {
        mScene->simulate(1.0f / 60.0f);
        mScene->fetchResults(true);
    }

    // cleanup
    mScene->release();
    mDispatcher->release();
    mPhysics->release();
    mPvd->release();
    transport->release();
    mFoundation->release();

    return 0;
}