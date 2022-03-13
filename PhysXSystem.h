#pragma once
#include "../../Plugins/physx/include/PxPhysicsAPI.h"
#include <string>
#include "../../Common/Vector3.h"
#include <OGLMesh.h>
#include <OGLRenderer.h>

/*
Link to Documentation:

https://docs.google.com/document/d/15Zsw6ItS5m1q5gB5k9gF6DJSgR7JpQIrI-aWnQrtdZA/edit?usp=sharing
*/

namespace NCL {
	namespace CSC8503 {
		class PhysXSystem {
		public:
			PhysXSystem();
			~PhysXSystem();

			void Update(float dt);

			void AddActor(physx::PxActor* actor);
			physx::PxActor* AddActor(bool isDynamic, const Maths::Vector3& size, const Maths::Vector3& position, float mass, std::string shape, float friction = 0.5f, float restitution = 0.1f); //Old, avoid using
			physx::PxActor* CreateCubeActor(const Maths::Vector3& position, const Maths::Vector3& size, float mass, float friction = 0.5f, float restitution = 0.1f);
			physx::PxActor* CreateCapsuleActor(const Maths::Vector3& position,float halfHeight,float radius, float mass, float friction = 0.5f, float restitution = 0.1f);
			physx::PxActor* CreateSphereActor(const Maths::Vector3& position, float radius, float mass, float friction = 0.5f, float restitution = 0.1f);
			physx::PxActor* CreateMeshActor(const Maths::Vector3& position, const Maths::Vector3& size, OGLMesh* mesh, float friction = 0.5f, float restitution = 0.1f);
			physx::PxActor* CreateWall(const Maths::Vector3& position, const Maths::Vector3& size, float friction = 0.5f, float restitution = 0.1f);

			void RemoveActor(physx::PxActor* actor);

			void Clear() {
				ClearPhysXScene();
			}

			void SetGravity(bool g);
		protected:
			void InitialisePhysXScene();
			void ClearPhysXScene();
			physx::PxTriangleMesh* ConvertToPxMesh(OGLMesh* mesh);

			physx::PxDefaultAllocator      phDefaultAllocatorCallback;
			physx::PxDefaultErrorCallback  phDefaultErrorCallback;
			physx::PxDefaultCpuDispatcher* phDispatcher = NULL;
			physx::PxTolerancesScale       phToleranceScale;

			physx::PxFoundation* phFoundation = NULL;
			physx::PxCooking* phCooking = NULL;
			physx::PxPhysics* physics;

			physx::PxScene* phScene = NULL;

			physx::PxPvd* phPvd = NULL;
		};
	}
}