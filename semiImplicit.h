#pragma once

#include "math3d.h"

int numberOfBodies = 0;
const int maxNumberOfBodies = 10;

int numberOfFloorPlanes = 0;
const int maxNumberOfFloorPlanes = 1;

//Declarations for semiImplicit.cc
//-------------------------------------------------------------------------------------------------------------------------------
//rigid_body struct -> object for each box.
struct rigidBody
{
	float width, height, length; //x, y, z lengths of box.
	float oneOverMass;
	Matrix oneOverBodySpaceInertiaTensor;
	float coefficientOfRestitution;
	
	//Size of the boxes. Defined by the number of bounding vertexes - 8 for a box.
	int unsigned boundingVertexes;
	enum{maxBoundingVertexes = 8};
	Vector BodyBoundingVertexes[maxBoundingVertexes];
	
	int reorthogonalizationRotor = 3;
	int &reorthRef = reorthogonalizationRotor;
	
	enum{maxConfigStates = 2};
	
	struct configuration
	{
		Vector CMposition;
		Matrix orientation;
		
		Vector CMvelocity;
		Vector angularMomentum;
		
		Vector CMforce;
		Vector torque;
		
		Matrix oneOverWorldSpaceInertiaTensor;
		Vector angularVelocity;
		
		Vector BodyBoundingVertexes[maxBoundingVertexes];
		
	}configState[maxConfigStates]; //These are object names - initial configuration and it's related values, and final config with it's related values.
};

struct floorPlane
{
	float width = 50.0, height = 50.0, length = 50.0; //x, y, z length of floor.
	float oneOverMass;
	Matrix orientation; //Used to set the inverse world inertia tensor
	Matrix oneOverBodySpaceInertiaTensor;
	Matrix oneOverWorldSpaceInertiaTensor;
	float coefficientOfRestitution = 1.0;
	
	Vector centrePosition;
	
	Vector planeNormal;
	float d;
};

//-------------------------------------------------------------------------------------------------------------------------------
//simulation_world object -> object for the world.
class simulation_world
{
	private:		
		
		//Collision part
		enum collision_state
		{
			penetrating, clear, colliding //If bodies are inside each other, just colliding, or not colliding at all
		} collisionState;
		
		Vector collisionNormal;
		int collidingBodyIndex;
		int collidingCornerIndex;
		
		collision_state checkForCollisions(int configurationIndex);
		void resolveCollisions(int collisionIndex);
		
		//Other functions
		void computeForces(int configState);
		
		void integrate(double DeltaTime);
		
		void calculateVertices(int configurationIndex);
		
		/*
		//If the floor is not a struct
		enum{numberOfWorldPlanes = 1};
		struct planes
		{
			Vector planeNormal;
			float d;
		}worldPlanes[numberOfWorldPlanes];
		*/
		
		int SourceConfigIndex;
		int TargetConfigIndex;
		
	public:
		simulation_world(float worldX, float worldY, float worldZ); //constructor
		~simulation_world(); //Destructor
		
		rigidBody BodyList[maxNumberOfBodies];
		void buildCubeBody(float Density, float cubeSize[3], float position[3], float coefficientOfRestitution);
		
		floorPlane floorList[maxNumberOfFloorPlanes];
		void buildFloor(float Mass);
		
		void Simulate(double DeltaTime);
		void render();
};	

