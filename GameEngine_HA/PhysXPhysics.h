//#pragma once
//#include <physx/PxPhysicsAPI.h>
//#include <glm/glm.hpp>
//#include <glm/gtc/quaternion.hpp>
//#include <glm/gtx/quaternion.hpp>
//#include <glm/gtx/euler_angles.hpp>
//#include <glm/gtx/norm.hpp>
//#include <vector>
//#include <map>
//class PhysXPhysics
//{
//public:
//	PhysXPhysics() = default;
//	~PhysXPhysics() = default;
//	void Initialize();
//	void ShutDown();
//	void Update();
//	void Reset();
//	void removeActor(physx::PxRigidActor* actorToDelete);
//	void createScene();
//	physx::PxRigidDynamic* createSphere(float radius, glm::vec3 position, float mass);
//	physx::PxRigidDynamic* createCube(float scale, glm::vec3 position, glm::quat rotation, float mass);
//	physx::PxRigidStatic* createStaticCube(float scale, glm::vec3 position, glm::quat rotation);
//	physx::PxRigidDynamic* createCylinder(float radius, float halfHeight, glm::vec3 position, glm::quat rotation, float mass);
//
//	physx::PxPhysics* mPhysics = NULL;
//	physx::PxScene* mScene = NULL;
//	physx::PxMaterial* mMaterial = NULL;
//private:
//	physx::PxDefaultAllocator      mDefaultAllocatorCallback;
//	physx::PxDefaultErrorCallback  mDefaultErrorCallback;
//	physx::PxDefaultCpuDispatcher* mDispatcher = NULL;
//	physx::PxTolerancesScale       mToleranceScale;
//
//	physx::PxFoundation* mFoundation = NULL;
//	physx::PxPvdTransport* mTransport;
//	physx::PxPvd* mPvd = NULL;
//	std::vector<physx::PxRigidActor*> mActors;
//	std::map<physx::PxRigidActor*, physx::PxTransform> mOriginalTransforms;
//};