//#pragma once
//#include "../cMeshObject.h"
//
//class GameObject
//{
//public:
//	GameObject() = default;
//	GameObject(physx::PxRigidDynamic* rigidBody, cMeshObject* mesh) { this->rigidBody = rigidBody, this->mesh = mesh; }
//	~GameObject() = default;
//	physx::PxRigidDynamic* rigidBody;
//	cMeshObject* mesh;
//};

#pragma once
#include <Interface/iRigidBody.h>
#include "../cMeshObject.h"
using namespace physics;
class GameObject
{
public:
	iRigidBody* rigidBody;
	cMeshObject* mesh;
};