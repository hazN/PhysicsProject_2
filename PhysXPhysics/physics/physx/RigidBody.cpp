#include "RigidBody.h"

#include "Conversion.h"

namespace physics
{
	RigidBody* RigidBody::Cast(iCollisionBody* body)
	{
		return dynamic_cast<RigidBody*>(body);
	}

	RigidBody::RigidBody(const RigidBodyDesc& desc, iShape* shape)
		: iRigidBody()
		, m_Shape(shape)
		, m_Position(desc.position)
		, m_Rotation(desc.rotation)
		, m_IsStatic(desc.isStatic)
		, m_LinearVelocity(desc.linearVelocity)
		, m_AngularVelocity(desc.angularVelocity)
		, m_Restitution(desc.restitution)
		, m_Friction(desc.friction)
		, m_LinearDamping(desc.linearDamping)
		, m_AngularDamping(desc.angularDamping)
	{
		if (m_IsStatic || desc.mass <= 0.f) {
			m_Mass = 0.f;
			m_InvMass = 0.f;
			m_IsStatic = true;
		}
		else
		{
			m_Mass = desc.mass;
			m_InvMass = 1.f / m_Mass;
		}
		// will add if statements to check shape
		physx::PxRigidDynamic* rigidActor;
		physx::PxTransform transform(physx::PxVec3(desc.position.x, desc.position.y, desc.position.z));
		if (m_Shape->GetShapeType() == ShapeType::Sphere)
			pShape = _world->mPhysics->createShape(physx::PxSphereGeometry(desc.mass), *_world->mMaterial);
		else if (m_Shape->GetShapeType() == ShapeType::Box)
		{
			//physx::PxVec3 halfExtents(scale / 2.f, scale / 2.f, scale / 2.f);
			//physx::PxTransform cubeTransform(physx::PxVec3(position.x, position.y, position.z), physx::PxQuat(rotation.x, rotation.y, rotation.z, rotation.w));
			//physx::PxShape* cubeShape = mPhysics->createShape(physx::PxBoxGeometry(halfExtents), *mMaterial);
			//physx::PxRigidDynamic* cubeActor = mPhysics->createRigidDynamic(cubeTransform);
			//cubeActor->attachShape(*cubeShape);
			//physx::PxRigidBodyExt::updateMassAndInertia(*cubeActor, mass);
			//mActors.push_back(cubeActor);
			//mOriginalTransforms.emplace(cubeActor, cubeTransform);
			//mScene->addActor(*cubeActor);
			//cubeShape->release();

		}
		rigidActor = _world->mPhysics->createRigidDynamic(transform);
		rigidActor->attachShape(*pShape);
		physx::PxRigidBodyExt::updateMassAndInertia(*rigidActor, desc.mass);
	}
	RigidBody::~RigidBody(void)
	{
	}

	bool RigidBody::IsStatic(void) const
	{
		return m_IsStatic;
	}

	void RigidBody::setWorld(PhysicsWorld* world)
	{
		_world = world;
	}

	iShape* RigidBody::GetShape(void)
	{
		return m_Shape;
	}

	//void RigidBody::GetWorldTransform(glm::mat4& transformOut)
	//{
	//}

	void RigidBody::GetPosition(glm::vec3& positionOut)
	{
		physx::PxTransform position = rigidBody->getGlobalPose();
		positionOut.x = position.p.x;
		positionOut.y = position.p.y;
		positionOut.z = position.p.z;
	}

	void RigidBody::SetPosition(const glm::vec3& positionIn)
	{
		// Can't.
	}

	void RigidBody::GetRotation(glm::quat& orientationOut)
	{
		physx::PxTransform rotation = rigidBody->getGlobalPose();
		orientationOut.w = rotation.q.w;
		orientationOut.x = rotation.q.x;
		orientationOut.y = rotation.q.y;
		orientationOut.z = rotation.q.z;
	}
	glm::vec3 RigidBody::GetGLMPosition()
	{
		physx::PxTransform position = rigidBody->getGlobalPose();
		glm::vec3 pos = glm::vec3(position.p.x, position.p.y, position.p.z);
	}
	void RigidBody::SetRotation(const glm::quat& orientationIn)
	{
		// Can't
	}

	void RigidBody::ApplyForce(const glm::vec3& force)
	{
		physx::PxVec3 forceVec = physx::PxVec3(force.x, force.y, force.z);
		// Make sure its dynamic and not static
		if (rigidBody->is<physx::PxRigidDynamic>())
		{
			physx::PxRigidDynamic* dynamicActor = (physx::PxRigidDynamic*)rigidBody;
			dynamicActor->addForce(forceVec, physx::PxForceMode::eACCELERATION);
		}
	}

	void RigidBody::ApplyForceAtPoint(const glm::vec3& force, const glm::vec3& relativePoint)
	{
	}

	void RigidBody::ApplyImpulse(const glm::vec3& impulse)
	{
	}

	void RigidBody::ApplyImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& relativePoint)
	{
	}

	void RigidBody::ApplyTorque(const glm::vec3& torque)
	{
	}

	void RigidBody::ApplyTorqueImpulse(const glm::vec3& torqueImpulse)
	{
	}

};