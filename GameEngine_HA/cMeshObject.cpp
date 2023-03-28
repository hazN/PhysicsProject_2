#include "cMeshObject.h"

//class cMeshObject
//{
//public:
//	cMeshObject();
//	std::string meshName;
//
//	glm::vec3 position;     // 0,0,0 (origin)
//	glm::vec3 rotation;     // 0,0,0 ration around each Euler axis
//	glm::vec4 colour;		// 0,0,0,1 (black)
//	float scale;
//	bool isWireframe;

cMeshObject::cMeshObject()
{
	this->position = glm::vec3(0.0f);

	//this->rotation = glm::vec3(0.0f);		// Euler angles
	// Set rotation to 0 degrees around each axis
	this->qRotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));

	// RGB and "alpha" (A) 0.0f = transparent
//	this->scale = 1.0f;
	this->SetUniformScale(1.0f);
	this->isWireframe = false;

	// This will mainly be used for non-lit debug objects, eventually
	this->RGBA_colour = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	this->bUse_RGBA_colour = false;

	// Make this white unless it's plastic or metal
	// Specualar (w) of 1 is "not" shiny
	this->specular_colour_and_power = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);

	this->bDoNotLight = false;

	this->bIsVisible = true;
	this->hasPhysicsObject = false;
	this->textureRatios[0] = 0.0f;
	this->textureRatios[1] = 0.0f;
	this->textureRatios[2] = 0.0f;
	this->textureRatios[3] = 0.0f;
	this->textureRatios[4] = 0.0f;
	this->textureRatios[5] = 0.0f;
	this->textureRatios[6] = 0.0f;
	this->textureRatios[7] = 0.0f;

	id = tid++;
}

cMeshObject* cMeshObject::findObjectByFriendlyName(std::string nameToFind, bool bSearchChildren /*=true*/)
{
	for (std::vector< cMeshObject* >::iterator itCurrentMesh = this->vecChildMeshes.begin();
		itCurrentMesh != this->vecChildMeshes.end();
		itCurrentMesh++)
	{
		cMeshObject* pCurrentMesh = *itCurrentMesh;

		if (pCurrentMesh->friendlyName == nameToFind)
		{
			// Early exit
			return pCurrentMesh;
		}

		// Search children too? 
		cMeshObject* pChildMesh = pCurrentMesh->findObjectByFriendlyName(nameToFind, bSearchChildren);

		if (pChildMesh)       /* NULL = 0 = false) */
		{
			return pChildMesh;
		}
	}//for (std::vector< cMeshObject* >::iterator

	// Didn't find it. How sad.
	return NULL;
}//cMeshObject* cMeshObject::findObjectByFriendlyName