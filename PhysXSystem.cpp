#include "PhysXSystem.h"
#include "../../Common/Vector3.h"
#include "../../Common/Maths.h"
#include <vector>

using namespace physx;

/* 
Link to Documentation:

https://docs.google.com/document/d/15Zsw6ItS5m1q5gB5k9gF6DJSgR7JpQIrI-aWnQrtdZA/edit?usp=sharing
*/

NCL::CSC8503::PhysXSystem::PhysXSystem()
{
	InitialisePhysXScene();
}

NCL::CSC8503::PhysXSystem::~PhysXSystem()
{
	if (phPvd) {
		ClearPhysXScene();
	}
}

void NCL::CSC8503::PhysXSystem::Update(float dt)
{
	if (phScene) {
		phScene->simulate(dt);
		phScene->fetchResults(true);
	}
}

void NCL::CSC8503::PhysXSystem::AddActor(physx::PxActor* actor)
{
	phScene->addActor(*actor);
}

physx::PxActor* NCL::CSC8503::PhysXSystem::AddActor(bool isDynamic, const Maths::Vector3& size, const Maths::Vector3& position, float mass, std::string shapestr, float friction, float restitution) //Old, do not use
{
	physx::PxTransform pos(physx::PxVec3(physx::PxReal(position.x), physx::PxReal(position.y), physx::PxReal(position.z)));
	physx::PxRigidStatic* staticObj = physics->createRigidStatic(pos);
	physx::PxRigidDynamic* dynamicObj = physics->createRigidDynamic(pos);

	physx::PxShape* shape;
	physx::PxMaterial* actorMaterial = physics->createMaterial(friction, friction, restitution);

	if (shapestr == "Box") {
		shape = physics->createShape(physx::PxBoxGeometry(size.x, size.y, size.z), *actorMaterial);
	}
	else if (shapestr == "Sphere") {
		shape = physics->createShape(physx::PxSphereGeometry(size.x), *actorMaterial);
	}
	else if (shapestr == "Capsule") {
		shape = physics->createShape(physx::PxCapsuleGeometry(size.x, size.z), *actorMaterial); 
		PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
		shape->setLocalPose(relativePose);
	} 
	else {
		throw "Invalid Shape";
	}

	if (isDynamic) {
		staticObj->release();
		dynamicObj->attachShape(*shape);
		dynamicObj->setMass(mass);
		dynamicObj->setMassSpaceInertiaTensor(PxVec3(mass));
		shape->release();
		dynamicObj->setName("Dynamic Test");
		phScene->addActor(*dynamicObj);

		return dynamicObj;
	}
	else {
		dynamicObj->release();
		staticObj->attachShape(*shape);
		shape->release();
		staticObj->setName("Static Test");
		phScene->addActor(*staticObj);

		return staticObj;
	}
}

physx::PxActor* NCL::CSC8503::PhysXSystem::CreateCubeActor(const Maths::Vector3& position, const Maths::Vector3& size, float mass, float friction, float restitution)
{
	physx::PxTransform pos(physx::PxVec3(physx::PxReal(position.x), physx::PxReal(position.y), physx::PxReal(position.z)));
	physx::PxRigidDynamic* dynamicObj = physics->createRigidDynamic(pos);
	physx::PxMaterial* actorMaterial = physics->createMaterial(friction, friction, restitution);
	physx::PxShape* shape = physics->createShape(physx::PxBoxGeometry(size.x, size.y, size.z), *actorMaterial);
	dynamicObj->attachShape(*shape);
	dynamicObj->setMass(mass);
	dynamicObj->setMassSpaceInertiaTensor(PxVec3(mass));
	shape->release();
	phScene->addActor(*dynamicObj);

	return dynamicObj;
}

physx::PxActor* NCL::CSC8503::PhysXSystem::CreateCapsuleActor(const Maths::Vector3& position, float halfHeight, float radius, float mass, float friction, float restitution)
{
	physx::PxTransform pos(physx::PxVec3(physx::PxReal(position.x), physx::PxReal(position.y), physx::PxReal(position.z)));
	physx::PxRigidDynamic* dynamicObj = physics->createRigidDynamic(pos);
	physx::PxMaterial* actorMaterial = physics->createMaterial(friction, friction, restitution);
	physx::PxShape* shape = physics->createShape(physx::PxCapsuleGeometry(radius, halfHeight), *actorMaterial);
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	shape->setLocalPose(relativePose);
	dynamicObj->attachShape(*shape);
	dynamicObj->setMass(mass);
	dynamicObj->setMassSpaceInertiaTensor(PxVec3(mass));
	shape->release();
	phScene->addActor(*dynamicObj);
	
	return dynamicObj;
}

physx::PxActor* NCL::CSC8503::PhysXSystem::CreateSphereActor(const Maths::Vector3& position, float radius, float mass, float friction, float restitution)
{
	physx::PxTransform pos(physx::PxVec3(physx::PxReal(position.x), physx::PxReal(position.y), physx::PxReal(position.z)));
	physx::PxRigidDynamic* dynamicObj = physics->createRigidDynamic(pos);
	physx::PxMaterial* actorMaterial = physics->createMaterial(friction, friction, restitution);
	physx::PxShape* shape = physics->createShape(physx::PxSphereGeometry(radius), *actorMaterial);
	dynamicObj->attachShape(*shape);
	dynamicObj->setMass(mass);
	dynamicObj->setMassSpaceInertiaTensor(PxVec3(mass));
	shape->release();
	phScene->addActor(*dynamicObj);
	
	return dynamicObj;
}

physx::PxActor* NCL::CSC8503::PhysXSystem::CreateMeshActor(const Maths::Vector3& position, const Maths::Vector3& size, OGLMesh* mesh, float friction, float restitution)
{
	physx::PxTransform pos(physx::PxVec3(physx::PxReal(position.x), physx::PxReal(position.y), physx::PxReal(position.z)));
	physx::PxRigidStatic* staticObj = physics->createRigidStatic(pos);
	physx::PxMaterial* actorMaterial = physics->createMaterial(friction, friction, restitution);
	PxTriangleMeshGeometry* newMesh = new PxTriangleMeshGeometry(ConvertToPxMesh(mesh),PxMeshScale(PxVec3(size.x,size.y,size.z)));
	physx::PxShape* shape = physics->createShape(*newMesh,*actorMaterial);
	staticObj->attachShape(*shape);
	shape->release();
	phScene->addActor(*staticObj);

	return staticObj;
}

physx::PxActor* NCL::CSC8503::PhysXSystem::CreateWall(const Maths::Vector3& position, const Maths::Vector3& size, float friction, float restitution)
{
	physx::PxTransform pos(physx::PxVec3(physx::PxReal(position.x), physx::PxReal(position.y), physx::PxReal(position.z)));
	physx::PxRigidStatic* staticObj = physics->createRigidStatic(pos);
	physx::PxMaterial* actorMaterial = physics->createMaterial(friction, friction, restitution);
	physx::PxShape* shape = physics->createShape(physx::PxBoxGeometry(size.x, size.y, size.z), *actorMaterial);
	staticObj->attachShape(*shape);
	shape->release();
	phScene->addActor(*staticObj);

	return staticObj;
}

void NCL::CSC8503::PhysXSystem::RemoveActor(physx::PxActor* actor)
{
	phScene->removeActor(*actor);
}

void NCL::CSC8503::PhysXSystem::SetGravity(bool g)
{
	PxU32 num = phScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
	if (num > 0) {
		std::vector<PxRigidDynamic*> actors(num);
		phScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, reinterpret_cast<PxActor**>(&actors[0]), num);

		for (PxRigidDynamic* actor : actors) {
			actor->wakeUp();
		}
	}

	if (g) {
		phScene->setGravity(physx::PxVec3(0.0f, -9.81f, 0.0f));
	}
	else {
		phScene->setGravity(physx::PxVec3(0.0f, 0.0f, 0.0f));
	}
}

void NCL::CSC8503::PhysXSystem::InitialisePhysXScene()
{
	phFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, phDefaultAllocatorCallback, phDefaultErrorCallback);
	if (!phFoundation) throw("PxCreateFoundation failed!");

	phPvd = PxCreatePvd(*phFoundation);
	physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	phPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);

	phToleranceScale.length = 1;        // typical length of an object
	phToleranceScale.speed = 9.81;         // typical speed of an object, gravity*1s is a reasonable choice
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *phFoundation, phToleranceScale, true, phPvd);

	phCooking = PxCreateCooking(PX_PHYSICS_VERSION, *phFoundation, PxCookingParams(phToleranceScale));
	if (!phCooking) throw("PxCreateCooking failed!");

	physx::PxSceneDesc sceneDesc(physics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	phDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = phDispatcher;
	sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
	phScene = physics->createScene(sceneDesc);
	phScene->setFlag(PxSceneFlag::eENABLE_ACTIVE_ACTORS, true);

	physx::PxPvdSceneClient* pvdClient = phScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
}

void NCL::CSC8503::PhysXSystem::ClearPhysXScene()
{
	phScene->release();
	physics->release();
	phFoundation->release();
	phPvd->release();
	phCooking->release();
}

physx::PxTriangleMesh* NCL::CSC8503::PhysXSystem::ConvertToPxMesh(OGLMesh* mesh)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = mesh->GetVertexCount();
	meshDesc.points.stride = sizeof(Vector3);
	meshDesc.points.data = &mesh->GetPositionData()[0];
	meshDesc.triangles.count = mesh->GetIndexCount();
	meshDesc.triangles.stride = 3 * sizeof(uint32_t);
	meshDesc.triangles.data = &mesh->GetIndexData()[0];

	if (meshDesc.isValid()) {
		PxDefaultMemoryOutputStream writeBuffer;
		PxTriangleMeshCookingResult::Enum result;
		bool status = phCooking->cookTriangleMesh(meshDesc, writeBuffer, &result);
		if (!status)
			return NULL;

		PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
		return physics->createTriangleMesh(readBuffer);
	}

	return nullptr;
}
