#include "cLightManager.h"

#include "globalOpenGL.h"
#include <sstream>		// Light a string builder in other languages

cLightManager::cLightManager()
{
	// Create the initial lights
	for ( unsigned int count = 0;
		  count != cLightManager::NUMBER_OF_LIGHTS_IM_GONNA_USE;
		  count++ )
	{

		cLight newLight;
		this->vecTheLights.push_back(newLight);
	}

}

void cLightManager::LoadLightUniformLocations(unsigned int shaderID)
{
//	unsigned int lightNumber = 0;
//	for (std::vector<cLight>::iterator itLight = this->vecTheLights.begin();
//		 itLight != this->vecTheLights.end();
//		 itLight++ )
//	{
////		std::string lightName = "theLights[" + lightNumber + "].position";
//		std::stringstream ssLightName;
//		ssLightName << "theLights[" << lightNumber << "].";
//
//		// Set the light information for lights (from the shader)
//		std::string lightPosition = ssLightName.str() + "position";
//		itLight->position_UniformLocation = glGetUniformLocation(shaderID, lightPosition.c_str() );
//
//		// Left for the student to complete... 
//		itLight->diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].diffuse");
//		itLight->specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].specular");
//		itLight->atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].atten");
//		itLight->direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].direction");
//		itLight->param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].param1");
//		itLight->param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].param2");
//
//		lightNumber++;
//	}

	// Or you could do this:
	this->vecTheLights[0].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].position");
	this->vecTheLights[0].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].diffuse");
	this->vecTheLights[0].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].specular");
	this->vecTheLights[0].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].atten");
	this->vecTheLights[0].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].direction");
	this->vecTheLights[0].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].param1");
	this->vecTheLights[0].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[0].param2");

	this->vecTheLights[1].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[1].position");
	this->vecTheLights[1].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[1].diffuse");
	this->vecTheLights[1].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[1].specular");
	this->vecTheLights[1].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[1].atten");
	this->vecTheLights[1].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[1].direction");
	this->vecTheLights[1].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[1].param1");
	this->vecTheLights[1].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[1].param2");

	this->vecTheLights[2].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[2].position");
	this->vecTheLights[2].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[2].diffuse");
	this->vecTheLights[2].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[2].specular");
	this->vecTheLights[2].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[2].atten");
	this->vecTheLights[2].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[2].direction");
	this->vecTheLights[2].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[2].param1");
	this->vecTheLights[2].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[2].param2");

	this->vecTheLights[3].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[3].position");
	this->vecTheLights[3].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[3].diffuse");
	this->vecTheLights[3].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[3].specular");
	this->vecTheLights[3].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[3].atten");
	this->vecTheLights[3].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[3].direction");
	this->vecTheLights[3].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[3].param1");
	this->vecTheLights[3].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[3].param2");

	this->vecTheLights[4].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[4].position");
	this->vecTheLights[4].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[4].diffuse");
	this->vecTheLights[4].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[4].specular");
	this->vecTheLights[4].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[4].atten");
	this->vecTheLights[4].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[4].direction");
	this->vecTheLights[4].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[4].param1");
	this->vecTheLights[4].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[4].param2");

	this->vecTheLights[5].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[5].position");
	this->vecTheLights[5].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[5].diffuse");
	this->vecTheLights[5].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[5].specular");
	this->vecTheLights[5].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[5].atten");
	this->vecTheLights[5].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[5].direction");
	this->vecTheLights[5].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[5].param1");
	this->vecTheLights[5].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[5].param2");

	this->vecTheLights[6].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[6].position");
	this->vecTheLights[6].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[6].diffuse");
	this->vecTheLights[6].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[6].specular");
	this->vecTheLights[6].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[6].atten");
	this->vecTheLights[6].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[6].direction");
	this->vecTheLights[6].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[6].param1");
	this->vecTheLights[6].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[6].param2");

	this->vecTheLights[7].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[7].position");
	this->vecTheLights[7].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[7].diffuse");
	this->vecTheLights[7].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[7].specular");
	this->vecTheLights[7].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[7].atten");
	this->vecTheLights[7].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[7].direction");
	this->vecTheLights[7].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[7].param1");
	this->vecTheLights[7].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[7].param2");

	this->vecTheLights[8].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[8].position");
	this->vecTheLights[8].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[8].diffuse");
	this->vecTheLights[8].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[8].specular");
	this->vecTheLights[8].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[8].atten");
	this->vecTheLights[8].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[8].direction");
	this->vecTheLights[8].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[8].param1");
	this->vecTheLights[8].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[8].param2");

	this->vecTheLights[9].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[9].position");
	this->vecTheLights[9].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[9].diffuse");
	this->vecTheLights[9].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[9].specular");
	this->vecTheLights[9].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[9].atten");
	this->vecTheLights[9].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[9].direction");
	this->vecTheLights[9].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[9].param1");
	this->vecTheLights[9].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[9].param2");

	this->vecTheLights[10].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[10].position");
	this->vecTheLights[10].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[10].diffuse");
	this->vecTheLights[10].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[10].specular");
	this->vecTheLights[10].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[10].atten");
	this->vecTheLights[10].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[10].direction");
	this->vecTheLights[10].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[10].param1");
	this->vecTheLights[10].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[10].param2");

	this->vecTheLights[11].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[11].position");
	this->vecTheLights[11].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[11].diffuse");
	this->vecTheLights[11].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[11].specular");
	this->vecTheLights[11].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[11].atten");
	this->vecTheLights[11].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[11].direction");
	this->vecTheLights[11].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[11].param1");
	this->vecTheLights[11].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[11].param2");

	this->vecTheLights[12].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[12].position");
	this->vecTheLights[12].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[12].diffuse");
	this->vecTheLights[12].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[12].specular");
	this->vecTheLights[12].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[12].atten");
	this->vecTheLights[12].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[12].direction");
	this->vecTheLights[12].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[12].param1");
	this->vecTheLights[12].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[12].param2");

	this->vecTheLights[13].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[13].position");
	this->vecTheLights[13].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[13].diffuse");
	this->vecTheLights[13].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[13].specular");
	this->vecTheLights[13].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[13].atten");
	this->vecTheLights[13].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[13].direction");
	this->vecTheLights[13].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[13].param1");
	this->vecTheLights[13].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[13].param2");

	this->vecTheLights[14].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[14].position");
	this->vecTheLights[14].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[14].diffuse");
	this->vecTheLights[14].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[14].specular");
	this->vecTheLights[14].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[14].atten");
	this->vecTheLights[14].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[14].direction");
	this->vecTheLights[14].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[14].param1");
	this->vecTheLights[14].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[14].param2");

	this->vecTheLights[15].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[15].position");
	this->vecTheLights[15].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[15].diffuse");
	this->vecTheLights[15].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[15].specular");
	this->vecTheLights[15].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[15].atten");
	this->vecTheLights[15].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[15].direction");
	this->vecTheLights[15].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[15].param1");
	this->vecTheLights[15].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[15].param2");

	this->vecTheLights[16].position_UniformLocation = glGetUniformLocation(shaderID, "theLights[16].position");
	this->vecTheLights[16].diffuse_UniformLocation = glGetUniformLocation(shaderID, "theLights[16].diffuse");
	this->vecTheLights[16].specular_UniformLocation = glGetUniformLocation(shaderID, "theLights[16].specular");
	this->vecTheLights[16].atten_UniformLocation = glGetUniformLocation(shaderID, "theLights[16].atten");
	this->vecTheLights[16].direction_UniformLocation = glGetUniformLocation(shaderID, "theLights[16].direction");
	this->vecTheLights[16].param1_UniformLocation = glGetUniformLocation(shaderID, "theLights[16].param1");
	this->vecTheLights[16].param2_UniformLocation = glGetUniformLocation(shaderID, "theLights[16].param2");
	return;
}

void cLightManager::CopyLightInformationToShader(unsigned int shaderID)
{
	for (std::vector<cLight>::iterator itLight = this->vecTheLights.begin();
		 itLight != this->vecTheLights.end();
		 itLight++)
	{
		glUniform4f(itLight->position_UniformLocation, 
					itLight->position.x, 
					itLight->position.y, 
					itLight->position.z, 
					itLight->position.w );

		glUniform4f(itLight->diffuse_UniformLocation,
					itLight->diffuse.x,
					itLight->diffuse.y,
					itLight->diffuse.z,
					itLight->diffuse.w );

		glUniform4f(itLight->specular_UniformLocation,
					itLight->specular.x,
					itLight->specular.y,
					itLight->specular.z,
					itLight->specular.w );

		glUniform4f(itLight->atten_UniformLocation,
					itLight->atten.x,
					itLight->atten.y,
					itLight->atten.z,
					itLight->atten.w );

		glUniform4f(itLight->direction_UniformLocation,
					itLight->direction.x,
					itLight->direction.y,
					itLight->direction.z,
					itLight->direction.w );

		glUniform4f(itLight->param1_UniformLocation,
					itLight->param1.x,
					itLight->param1.y,
					itLight->param1.z,
					itLight->param1.w );

		glUniform4f(itLight->param2_UniformLocation,
					itLight->param2.x,
					itLight->param2.y,
					itLight->param2.z,
					itLight->param2.w );


	}// for (std::vector<cLight>::iterator itLight

	return;
}

