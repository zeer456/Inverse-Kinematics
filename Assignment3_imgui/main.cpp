#define _CRT_SECURE_NO_WARNINGS
#define STB_IMAGE_IMPLEMENTATION

#include "stb_image.h"

#include <stdio.h>
#include <cstdlib>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <GL/glew.h>   
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <glm/gtc/type_ptr.hpp>

#include "opengl_utils.h"
#include "CGobject.h"

#include "..\Dependencies\OBJ_Loader.h"
#include "Interpolate.h"

// Macro for indexing vertex buffer
#define BUFFER_OFFSET(i) ((char *)NULL + (i))
#define MAX_OBJECTS 50
#define MAX_IK_TRIES 100 // TIMES THROUGH THE CCD LOOP
#define IK_POS_THRESH 0.01f // THRESHOLD FOR SUCCESS
#define MAX_KEYFRAMES_GOAL 8 

using namespace glm;
using namespace std;
using namespace Assignment3_imgui;

// GLFW 
GLFWwindow* window;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_pos_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1100;

opengl_utils glutils;

bool lbutton_down = false;

// timing
float deltaTimeBetweenFrames = 0.0f;	// time between current frame and last frame
float lastFrameTime = 0.0f;

// camera movement
bool firstMouse = true;
float myyaw = -90.0f;	// yaw is initialized to -90.0 degrees
float mypitch = 0.0f;
float lastX = SCR_WIDTH / 2.0; //800.0f / 2.0;
float lastY = SCR_HEIGHT / 2.0; //600.0 / 2.0;
float fov = 45.0f;

// camera
glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 15.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

float initialCylinderLength = 2.0f;

GLuint VAOs[MAX_OBJECTS];
int numVAOs = 0;

GLuint textures[4];

unsigned int textureIDcubemap;
unsigned int textureIDlotus;
unsigned int textureIDlotusBump;

unsigned int n_vbovertices = 0;
unsigned int n_ibovertices = 0;

CGObject sceneObjects[MAX_OBJECTS];
int numObjects = 0;


// ImGUI definitions
bool show_demo_window = true;
bool show_another_window = false;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
bool withinReach = false;

float keyframeGoalPositions[MAX_KEYFRAMES_GOAL][3];
Curve* curve;

enum IKMethod
{
	CCD
};

enum AnimType
{
	track
};

enum AnimSpeed
{
	no_speed,
	equal,
	easeIn_easeOut
};

int IKmethod = IKMethod::CCD;
int animType = AnimType::track;
bool animStarted = false;
float animStartTime = 0.0f;
int animCurrFrame = 0;
int animationSpeed = AnimSpeed::no_speed;
float animationLength = 10.0f;  // 20 sec
float equalSpeed = 0.0;
float acceleration = 0.0;
float deceleration = 0.0;
float timePerLineSegment = 3.0f;

int numJoints = 2;
float goal[3] = { 8.0f,0.0f,0.0f };
float increment = 0.1f;

int sphereIndex = 1;
int indexOfFirstJoint = 3;


static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

//lighting position
glm::vec3 lightPos(3.0f, 1.0f, 3.0f);

enum class textureInterpolation
{
	nearest = 1,
	linear = 2,
	nearest_mipmap_nearest_interpolation = 3, 
	nearest_mipmap_linear_interpolation = 4,
	interpolate_mipmap_nearest_interpolation = 5, 
	interpolate_mipmap_linear_interpolation = 6, 
};

void addToObjectBuffer(CGObject* cg_object)
{
	int VBOindex = cg_object->startVBO;
	int IBOindex = cg_object->startIBO;

	for (int i = 0; i < cg_object->Meshes.size(); i++) {

		glutils.addVBOBufferSubData(VBOindex, cg_object->Meshes[i].Vertices.size(), &cg_object->Meshes[i].Vertices[0].Position.X);
		VBOindex += cg_object->Meshes[i].Vertices.size();
		IBOindex += cg_object->Meshes[i].Indices.size();
	}
}

void addToTangentBuffer(CGObject* cg_object)
{
	int VBOindex = cg_object->startVBO;
	int IBOindex = cg_object->startIBO;

	for (int i = 0; i < cg_object->Meshes.size(); i++) {

		glutils.addTBOBufferSubData(VBOindex, cg_object->tangentMeshes[i].tangents.size(), &cg_object->tangentMeshes[i].tangents[0].x);
		VBOindex += cg_object->Meshes[i].Vertices.size();
		IBOindex += cg_object->Meshes[i].Indices.size();
	}
}

void addToBitangentBuffer(CGObject* cg_object)
{
	int VBOindex = cg_object->startVBO;
	int IBOindex = cg_object->startIBO;

	for (int i = 0; i < cg_object->Meshes.size(); i++) {

		glutils.addBTBOBufferSubData(VBOindex, cg_object->tangentMeshes[i].bitangents.size(), &cg_object->tangentMeshes[i].bitangents[0].x);
		VBOindex += cg_object->Meshes[i].Vertices.size();
		IBOindex += cg_object->Meshes[i].Indices.size();
	}
}

void addToIndexBuffer(CGObject* cg_object)
{
	int IBOindex = cg_object->startIBO;
	for (auto const& mesh : cg_object->Meshes) {
		glutils.addIBOBufferSubData(IBOindex, mesh.Indices.size(), &mesh.Indices[0]);
		IBOindex += mesh.Indices.size();
	}
}


std::vector<objl::Mesh> loadMeshes(const char* objFileLocation)
{
	objl::Loader obj_loader;

	bool result = obj_loader.LoadFile(objFileLocation);
	if (result && obj_loader.LoadedMeshes.size() > 0)
	{
		return obj_loader.LoadedMeshes;
	}
	else
		throw new exception("Could not load mesh");
}

CGObject loadObjObject(vector<objl::Mesh> meshes, vector<TangentMesh> tangentMeshes, bool addToBuffers, bool subjectToGravity, vec3 initTransformVector, vec3 initScaleVector, vec3 color, float coef, CGObject* parent)
{
	CGObject object = CGObject();
	object.Meshes = meshes;
	object.tangentMeshes = tangentMeshes;
	object.subjectToGravity = subjectToGravity;
	object.initialTranslateVector = initTransformVector;
	object.position = initTransformVector;
	object.initialScaleVector = initScaleVector;
	object.color = color;
	object.coef = coef;
	object.Parent = parent;
	object.startVBO = n_vbovertices;
	object.startIBO = n_ibovertices;
	object.VAOs = std::vector<GLuint>();

	if (addToBuffers)
	{
		for (auto const& mesh : meshes) {
			glutils.generateVertexArray(&(VAOs[numVAOs]));
			GLuint tmpVAO = VAOs[numVAOs];
			object.VAOs.push_back(tmpVAO);
			n_vbovertices += mesh.Vertices.size();
			n_ibovertices += mesh.Indices.size();
			numVAOs++;
		}
	}

	return object;
}

CGObject createTorso(int& numberOfObjects)
{
	const char* cylinderFileName = "../Assignment3_imgui/meshes/Cylinder/cylinder.obj";
	vector<objl::Mesh> cylinderMeshes = loadMeshes(cylinderFileName);

	std::vector<objl::Mesh> new_meshesCylinder;
	std::vector<TangentMesh> new_tangentMeshesCylinder;

	//recalculate meshes
	CGObject::recalculateVerticesAndIndexes(cylinderMeshes, new_meshesCylinder, new_tangentMeshesCylinder);

	CGObject torso = loadObjObject(new_meshesCylinder, new_tangentMeshesCylinder, true, true, vec3(-2.0f, -2.0f, 0.0f), vec3(7.0f, 1.0f, 7.0f), vec3(244 / 255.0f, 164 / 255.0f, 96 / 255.0f), 0.65f, NULL);
	sceneObjects[numberOfObjects] = torso;
	numberOfObjects++;

	return torso;
}

CGObject createCubeMap(int& numberOfObjects)
{
	// CUBEMAP
	const char* cubeFileName = "../Assignment3_imgui/meshes/Cube/Cube.obj";
	vector<objl::Mesh> cubeMesh = loadMeshes(cubeFileName);

	std::vector<objl::Mesh> dummy_cubeMeshes = std::vector<objl::Mesh>();
	std::vector<TangentMesh> dummy_cubeTangents = std::vector<TangentMesh>();
	CGObject::recalculateVerticesAndIndexes(cubeMesh, dummy_cubeMeshes, dummy_cubeTangents);

	CGObject cubeObject = loadObjObject(dummy_cubeMeshes, dummy_cubeTangents, true, true, vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 1.0f, 1.0f), vec3(1.0f, 1.0f, 1.0f), 0.65f, NULL);
	sceneObjects[numberOfObjects] = cubeObject;
	numberOfObjects++;

	return cubeObject;
}

void createObjects()
{
	// Shader Attribute locations
	glutils.getAttributeLocations();

	CGObject cubeObject = createCubeMap(numObjects);

	const char* sphereFileName = "../Assignment3_imgui/meshes/Sphere/sphere.obj";
	vector<objl::Mesh> sphereMeshes = loadMeshes(sphereFileName);

	std::vector<objl::Mesh> new_meshesSphere;
	std::vector<TangentMesh> new_tangentMeshesSphere;

	//recalculate meshes
	CGObject::recalculateVerticesAndIndexes(sphereMeshes, new_meshesSphere, new_tangentMeshesSphere);

	CGObject sphereObject = loadObjObject(new_meshesSphere, new_tangentMeshesSphere, true, true, vec3(5.0f, 0.0f, 0.0f), vec3(0.4f, 0.4f, 0.4f), vec3(0.0f, 1.0f, 0.0f), 0.65f, NULL);
	sphereObject.initialScaleVector = vec3(0.5f, 0.5f, 0.5f);
	sceneObjects[numObjects] = sphereObject;
	numObjects++;

	const char* cylinderFileName = "../Assignment3_imgui/meshes/Cylinder/cylinder_sm.obj"; 
	vector<objl::Mesh> cylinderMeshes = loadMeshes(cylinderFileName);

	std::vector<objl::Mesh> new_meshesCylinder;
	std::vector<TangentMesh> new_tangentMeshesCylinder;

	//recalculate meshes
	CGObject::recalculateVerticesAndIndexes(cylinderMeshes, new_meshesCylinder, new_tangentMeshesCylinder);

	CGObject torso = loadObjObject(new_meshesCylinder, new_tangentMeshesCylinder, true, true, vec3(-0.4f, -4.5f, 0.0f), vec3(7.0f, 2.5f, 7.0f), vec3(244 / 255.0f, 164 / 255.0f, 96 / 255.0f), 0.65f, NULL);
	sceneObjects[numObjects] = torso;
	numObjects++;

	CGObject topArm = loadObjObject(new_meshesCylinder, new_tangentMeshesCylinder, false, true, vec3(0.0f, 0.0f, 0.0f), vec3(1.5f, 1.0f, 1.5f), vec3(1.0f, 1.0f, 0.0f), 0.65f, NULL);
	topArm.setInitialRotation(vec3(0.0f, 0.0f, -2.5f));
	topArm.startVBO = torso.startVBO;  //reusing model
	topArm.startIBO = torso.startIBO;  //reusing model
	topArm.VAOs.push_back(torso.VAOs[0]);  //reusing model
	sceneObjects[numObjects] = topArm;
	numObjects++;

	initialCylinderLength = (topArm.Meshes[0].Vertices[1].Position.Y - topArm.Meshes[0].Vertices[0].Position.Y);

	CGObject bottomArm = loadObjObject(new_meshesCylinder, new_tangentMeshesCylinder, false, true,
		topArm.rotationMatrix * vec4(0.0, topArm.initialScaleVector.y * initialCylinderLength, 0.0, 1.0),
		vec3(1.5f, 0.8f, 1.5f),
		vec3(1.0f, 1.0f, 0.0f),
		0.65f,
		NULL);

	bottomArm.setInitialRotation(vec3(0.0f, 0.0f, -2.0f));
	bottomArm.startVBO = torso.startVBO;  //reusing model
	bottomArm.startIBO = torso.startIBO;  //reusing model
	bottomArm.VAOs.push_back(torso.VAOs[0]);  //reusing model
	sceneObjects[numObjects] = bottomArm;
	numObjects++;

	CGObject palm = loadObjObject(new_meshesCylinder, new_tangentMeshesCylinder, false, true,
		bottomArm.rotationMatrix * vec4(0.0, bottomArm.initialScaleVector.y * initialCylinderLength, 0.0, 1.0),
		vec3(1.5f, 0.3f, 1.5f),
		vec3(1.0f, 1.0f, 0.0f),
		0.65f,
		NULL);

	palm.setInitialRotation(vec3(0.0f, 0.0f, -1.5f));
	palm.startVBO = torso.startVBO;  //reusing model
	palm.startIBO = torso.startIBO;  //reusing model
	palm.VAOs.push_back(torso.VAOs[0]);  //reusing model
	sceneObjects[numObjects] = palm;
	numObjects++;

	CGObject fingers = loadObjObject(new_meshesCylinder, new_tangentMeshesCylinder, false, true,
		palm.rotationMatrix * vec4(0.0, palm.initialScaleVector.y * initialCylinderLength, 0.0, 1.0),
		vec3(1.5f, 0.15f, 1.5f),
		vec3(1.0f, 1.0f, 0.0f),
		0.65f,
		NULL);

	fingers.setInitialRotation(vec3(0.0f, 0.0f, -1.0f));
	fingers.startVBO = torso.startVBO;  //reusing model
	fingers.startIBO = torso.startIBO;  //reusing model
	fingers.VAOs.push_back(torso.VAOs[0]);  //reusing model
	sceneObjects[numObjects] = fingers;
	numObjects++;

	CGObject snake = loadObjObject(new_meshesCylinder, new_tangentMeshesCylinder, false, true,
		fingers.rotationMatrix * vec4(0.0, fingers.initialScaleVector.y * initialCylinderLength, 0.0, 1.0),
		vec3(1.5f, 0.15f, 1.5f),
		vec3(1.0f, 1.0f, 0.0f),
		0.65f,
		NULL);

	snake.setInitialRotation(vec3(0.0f, 0.0f, -0.5f));
	snake.startVBO = torso.startVBO;  //reusing model
	snake.startIBO = torso.startIBO;  //reusing model
	snake.VAOs.push_back(torso.VAOs[0]);  //reusing model
	sceneObjects[numObjects] = snake;
	numObjects++;

	glutils.createVBO(n_vbovertices);

	glutils.createIBO(n_ibovertices);

	addToObjectBuffer(&cubeObject);
	addToObjectBuffer(&sphereObject);
	addToObjectBuffer(&torso);
	addToIndexBuffer(&cubeObject);
	addToIndexBuffer(&sphereObject);
	addToIndexBuffer(&torso);

	glutils.createTBO(n_vbovertices);
	addToTangentBuffer(&cubeObject);
	addToTangentBuffer(&sphereObject);
	addToTangentBuffer(&torso);

	glutils.createBTBO(n_vbovertices);
	addToBitangentBuffer(&cubeObject);
	addToBitangentBuffer(&sphereObject);
	addToBitangentBuffer(&torso);
}

void loadCube()
{
	vector<std::string> faces = vector<std::string>();

	faces.push_back("../Assignment3_imgui/meshes/Yokohama/posx.jpg");
	faces.push_back("../Assignment3_imgui/meshes/Yokohama/negx.jpg");
	faces.push_back("../Assignment3_imgui/meshes/Yokohama/posy.jpg");
	faces.push_back("../Assignment3_imgui/meshes/Yokohama/negy.jpg");
	faces.push_back("../Assignment3_imgui/meshes/Yokohama/posz.jpg");
	faces.push_back("../Assignment3_imgui/meshes/Yokohama/negz.jpg");

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
				0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
			);
			stbi_image_free(data);
		}
		else
		{
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
}

//void readKeyframePositionsFile(string file)
//{
//
//	ifstream myReadFile;
//	int lineNumber = 0;  //
//	int weightIndex = 0;
//
//	myReadFile.open(file);
//
//	char output[100];
//	if (myReadFile.is_open()) {
//		while (!myReadFile.eof()) {
//
//			myReadFile >> output;
//
//			float d2 = strtod(output, NULL);
//			keyframeGoalPositions[lineNumber][weightIndex] = d2;
//			weightIndex++;
//
//			if (weightIndex == 3)
//			{
//				lineNumber++;
//				weightIndex = 0;
//			}
//		}
//	}
//	myReadFile.close();
//}

void populateAnimationCurve()
{
	curve = new Curve();
	curve->set_steps(100); // generate 100 interpolate points between the last 4 way points

	for (int i = 0; i < MAX_KEYFRAMES_GOAL; i++)
	{
		curve->add_way_point(glm::vec3(keyframeGoalPositions[i][0], keyframeGoalPositions[i][1], keyframeGoalPositions[i][2]));
	}

	equalSpeed = curve->total_length() / animationLength;
	acceleration = 4 * curve->total_length() / (animationLength * animationLength);  // for half length	
	deceleration = 4 * (curve->total_length() - (acceleration * animationLength * animationLength / 2)) / (animationLength * animationLength); 

	std::cout << "nodes: " << curve->node_count() << std::endl;
	std::cout << "total length: " << curve->total_length() << std::endl;
	
}

void init()
{
	populateAnimationCurve();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);// you enable blending function
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glutils = opengl_utils();

	glutils.createShaders();

	glutils.setupUniformVariables();

	// Setup cubemap texture
	glGenTextures(4, textures); 

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textures[0]); 

	loadCube();

	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glUniform1i(glutils.cubeLocation2, 0);   // cubemap
	glUniform1i(glutils.cubeLocation3, 0);   // cubemap

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textures[1]);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glUniform1i(glutils.texture3, 1);

	createObjects();
}

float sq_distance(vec3 point1, vec3 point2)
{
	return pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) + pow(point2.z - point1.z, 2);
}

bool computeCCDLink(glm::vec3 goalVec)
{
	bool goalWithinThreshold = false;

	int link = numJoints - 1;  
	int tries = 0;

	glm::vec3 currRoot, currEnd, lastJointPosition, targetVector, currVector, normal;
	float cosAngle, turnAngle, turnDegrees, lastJointLength;
	quat quaternion;
	int indexOfCurrLink, indexOfLastJoint;

	indexOfLastJoint = indexOfFirstJoint + numJoints - 1;

	do
	{
		indexOfCurrLink = indexOfFirstJoint + link;

		currRoot = sceneObjects[indexOfCurrLink].position;

		currEnd = sceneObjects[indexOfLastJoint].position + vec3(sceneObjects[indexOfLastJoint].rotationMatrix *
			vec4(0.0, sceneObjects[indexOfLastJoint].initialScaleVector.y * initialCylinderLength, 0.0, 1.0));

		if (sq_distance(goalVec, currEnd) < IK_POS_THRESH)
		{
			// we are close
			goalWithinThreshold = true;
		}
		else
		{
			// continue search
			currVector = currEnd - currRoot;
			targetVector = goalVec - currRoot;

			currVector = glm::normalize(currVector);
			targetVector = glm::normalize(targetVector);

			cosAngle = dot(targetVector, currVector);
			if (abs(cosAngle) < 0.99999)
			{
				normal = cross(currVector, targetVector);
				normal = glm::normalize(normal);
				turnAngle = acos(cosAngle);

				quaternion.x = normal.x * sin(turnAngle / 2);
				quaternion.y = normal.y * sin(turnAngle / 2);
				quaternion.z = normal.z * sin(turnAngle / 2);
				quaternion.w = cos(turnAngle / 2);

				sceneObjects[indexOfCurrLink].rotationMatrix = glm::toMat4(quaternion) * sceneObjects[indexOfCurrLink].rotationMatrix;	

				// update position and angles for all links
				for (int k = indexOfCurrLink + 1; k < numObjects; k++)
				{
					if (k != indexOfFirstJoint) // ignore first joint
					{
						sceneObjects[k].position = sceneObjects[k - 1].position +
							vec3(sceneObjects[k - 1].rotationMatrix * vec4(0.0, sceneObjects[k - 1].initialScaleVector.y * initialCylinderLength, 0.0, 1.0));
					}

					sceneObjects[k].rotationMatrix = glm::toMat4(quaternion) * sceneObjects[k].rotationMatrix;
				}
			}

			if (--link < 0)
			{
				// move back to last link
				link = numJoints - 1;
			}
		}

	} while (++tries < 1000 && goalWithinThreshold == false);  //MAX_IK_TRIES

	return goalWithinThreshold;
}

void displayScene(glm::mat4 projection, glm::mat4 view)
{
	glUseProgram(glutils.ShaderWithTextureID);

	glm::mat4 local1(1.0f);
	local1 = glm::translate(local1, cameraPos);
	glm::mat4 global1 = local1;

	glutils.updateUniformVariablesReflectance(global1, view, projection);

	glUniform3f(glutils.viewPosLoc3, cameraPos.x, -cameraPos.y, cameraPos.z);
	glUniform3f(glutils.lightPosLoc3, lightPos.x, lightPos.y, lightPos.z);

	// DRAW objects
	for (int i = 1; i < numObjects - ((numObjects - 3) - numJoints); i++)
	{
		// update position based on previous joint
		if (i > indexOfFirstJoint)
		{
			sceneObjects[i].position = sceneObjects[i - 1].position +
				vec3(sceneObjects[i - 1].rotationMatrix * vec4(0.0, sceneObjects[i - 1].initialScaleVector.y * initialCylinderLength, 0.0, 1.0));

		}

		mat4 globalCGObjectTransform = sceneObjects[i].createTransform(true);
		glutils.updateUniformVariablesReflectance(globalCGObjectTransform, view);
		sceneObjects[i].globalTransform = globalCGObjectTransform; // keep current state		

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, textures[0]);
		glUniform1i(glutils.cubeLocation3, 0);

		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, textures[1]);
		glUniform1i(glutils.texture3, 1);

		glUniform3f(glutils.objectColorLoc3, sceneObjects[i].color.r, sceneObjects[i].color.g, sceneObjects[i].color.b);
		sceneObjects[i].Draw(glutils, glutils.ShaderWithTextureID);

		glDisable(GL_TEXTURE_2D);
	}
}

void displayCubeMap(glm::mat4 projection, glm::mat4 view)
{
	// First Draw cube map - sceneObjects[0]
	glDepthMask(GL_FALSE);
	glUseProgram(glutils.CubeMapID);

	glm::mat4 viewCube = glm::mat4(glm::mat3(view));
	glutils.updateUniformVariablesCubeMap(viewCube, projection);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textures[0]); //textureIDcubemap);

	glBindVertexArray(VAOs[0]);

	glutils.bindVertexAttribute(glutils.loc4, 3, sceneObjects[0].startVBO, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, glutils.IBO);

	glDrawElements(GL_TRIANGLES, sceneObjects[0].Meshes[0].Indices.size(), GL_UNSIGNED_INT, (void*)(sceneObjects[0].startIBO * sizeof(unsigned int)));

	glDepthMask(GL_TRUE);

}

void drawImgui(ImGuiIO& io)
{
	io.WantCaptureMouse = true;
	io.WantCaptureKeyboard = true;

	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	{

		ImGui::Begin("Inverse Kinematics");

		ImGui::Text("IK Method: CCD");
		ImGui::Text("Animation Method: Track");

		const char* items3[] = { "None", "Equal", "Accelerate" };
		ImGui::Combo("Animation Speed", &animationSpeed, items3, IM_ARRAYSIZE(items3));

		if (ImGui::Button("Show Animation"))
		{
			animStarted = true;
			animStartTime = glfwGetTime();
			animCurrFrame = 0;

			if (animationSpeed == AnimSpeed::equal)
			{
				// divide distance by required time
				curve->total_length() / animationLength;

			}
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Select number of joints:");
		ImGui::SliderInt("Number of joints", &numJoints, 2, 5);

		ImGui::Text("Change the length of arm parts:");
		ImGui::DragFloat("Top arm", &sceneObjects[indexOfFirstJoint].initialScaleVector.y, 0.1f, 0.5f, 5.0f, "%.1f");
		ImGui::DragFloat("Bottom arm ", &sceneObjects[indexOfFirstJoint + 1].initialScaleVector.y, 0.1f, 0.5f, 5.0f, "%.1f");


		if (withinReach)
		{
			ImGui::Text("Object within reach");
		}
		else
		{
			ImGui::Text("Object outside of reach");
		}

		ImGui::End();
	}

	// Rendering
	int display_w, display_h;
	glfwMakeContextCurrent(window);
	glfwGetFramebufferSize(window, &display_w, &display_h);
	glViewport(0, 0, display_w, display_h);
	glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
	glClear(GL_COLOR_BUFFER_BIT);
	ImGui::Render();
}

void display(ImGuiIO& io)
{
	float currentFrameTime = glfwGetTime();
	deltaTimeBetweenFrames = currentFrameTime - lastFrameTime;
	lastFrameTime = currentFrameTime;

	// inpuT
	processInput(window);

	drawImgui(io);

	// render
	glClearColor(0.78f, 0.84f, 0.49f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Update projection 
	glm::mat4 projection = glm::perspective(glm::radians(fov), (float)(SCR_WIDTH) / (float)(SCR_HEIGHT), 0.1f, 100.0f);
	glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

	if (animStarted)
	{
		view = glm::lookAt(glm::vec3(0.0f, 0.0f, 15.0f), glm::vec3(0.0f, 0.0f, 15.0f) + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	}

	// DRAW CUBEMAP
	displayCubeMap(projection, view);

	// DRAW SCENE

	if (animStarted)
	{
		if (animationSpeed == no_speed)
		{
			// check for end of animation
			if (animCurrFrame < curve->node_count())
			{
				sceneObjects[sphereIndex].position.x = curve->node(animCurrFrame).x;
				sceneObjects[sphereIndex].position.y = curve->node(animCurrFrame).y;
				sceneObjects[sphereIndex].position.z = curve->node(animCurrFrame).z;
				animCurrFrame++;
			}
			else
			{
				animStarted = false;
				animStartTime = 0.0f;
				animCurrFrame = 0;
			}
		}
		else
		{
			float dt = currentFrameTime - animStartTime;

			if (dt < animationLength)
			{
				float distanceTravelled;

				if (animationSpeed == AnimSpeed::equal)
				{
					distanceTravelled = equalSpeed * dt;
				}
				else
				{

					//ease in out
					if (dt < 0.5f * animationLength)
					{
						// accelearate
						distanceTravelled = 0.5f * acceleration * dt * dt;
					}
					else
					{
						// decelearate
						dt = dt - 0.5f * animationLength;
						distanceTravelled = (curve->total_length() / 2) + (acceleration * animationLength / 2) * dt + (0.5f * deceleration * dt * dt);
					}

				}

				// find node that correponds to this distance - search from last frame
				for (int k = animCurrFrame; k < curve->node_count(); k++)
				{
					if (curve->length_from_starting_point(k) > distanceTravelled)
					{
						//found the right point
						sceneObjects[sphereIndex].position.x = curve->node(k).x;
						sceneObjects[sphereIndex].position.y = curve->node(k).y;
						sceneObjects[sphereIndex].position.z = curve->node(k).z;
						animCurrFrame = k;
						break;
					}
				}
			}
			else
			{
				// Reset
				animStarted = false;
				animStartTime = 0.0f;
				animCurrFrame = 0;
			}
		}
	}
	else
	{
		// Update target position from imGui
		sceneObjects[sphereIndex].position.x = goal[0];
		sceneObjects[sphereIndex].position.y = goal[1];
		sceneObjects[sphereIndex].position.z = goal[2];
	}

	displayScene(projection, view);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);

	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	withinReach = computeCCDLink(sceneObjects[sphereIndex].position);
	
	glfwSwapBuffers(window);
	glfwPollEvents();

}


int main(void) {
	// Initialise GLFW
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Inverse Kinematics", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window.\n");
		glfwTerminate();
		return -1;
	}

	glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, mouse_pos_callback);

	glfwSwapInterval(1); // Enable vsync

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		glfwTerminate();
		return -1;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableSetMousePos;

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	const char* glsl_version = "#version 330";
	ImGui_ImplOpenGL3_Init(glsl_version);

	init();

	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0)
	{
		display(io);
	}

	// optional: de-allocate all resources once they've outlived their purpose:	
	glutils.deleteVertexArrays();
	glutils.deletePrograms();
	glutils.deleteBuffers();
	delete curve;

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	float cameraSpeed = 2.5 * deltaTimeBetweenFrames;
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
	{
		cameraPos = glm::vec3(0.0f, 0.0f, 15.0f);
		cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
		cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
		firstMouse = true;
	}
	if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
	{
		cameraPos = glm::vec3(0.0f, 15.0f, 0.0f);
		cameraFront = glm::vec3(0.0f, -1.0f, 0.0f);
		cameraUp = glm::vec3(0.0f, 0.0f, 1.0f);
		firstMouse = true;
	}
	if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
	{
		cameraPos = glm::vec3(15.0f, 0.0f, 0.0f);
		cameraFront = glm::vec3(-1.0f, 0.0f, 0.0f);
		cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
		firstMouse = true;
	}
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
	{
		goal[1] += increment;
	}
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		goal[1] -= increment;
	}
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
	{
		goal[0] += increment;
	}
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
	{
		goal[0] -= increment;
	}
	if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS)
	{
		goal[2] += increment;
	}
	if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS)
	{
		goal[2] -= increment;
	}
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{

	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (GLFW_PRESS == action)
		{
			lbutton_down = true;
		}
		else if (GLFW_RELEASE == action)
		{
			lbutton_down = false;
			firstMouse = true;
		}
	}
}

void mouse_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (lbutton_down)
	{

		if (firstMouse)
		{
			lastX = xpos;
			lastY = ypos;
			firstMouse = false;
		}

		float xoffset = xpos - lastX;
		float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
		lastX = xpos;
		lastY = ypos;

		float sensitivity = 0.1f; 
		xoffset *= sensitivity;
		yoffset *= sensitivity;

		myyaw += xoffset;
		mypitch += yoffset;

		// make sure that when pitch is out of bounds, screen doesn't get flipped
		if (mypitch > 89.0f)
			mypitch = 89.0f;
		if (mypitch < -89.0f)
			mypitch = -89.0f;

		glm::vec3 front;
		front.x = cos(glm::radians(myyaw)) * cos(glm::radians(mypitch));
		front.y = sin(glm::radians(mypitch));
		front.z = sin(glm::radians(myyaw)) * cos(glm::radians(mypitch));
		cameraFront = glm::normalize(front);
	}
}

