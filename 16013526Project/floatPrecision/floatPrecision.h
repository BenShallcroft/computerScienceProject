#pragma once

#include "math3d.h"

int numberOfBodies = 0;
const int maxNumberOfBodies = 2;

int numberOfFloorPlanes = 0;
const int maxNumberOfFloorPlanes = 1;

int numberOfCollisions = 0;
const int maxNumberOfCollisions = 100;

int collidingIndexMasterNumber = 0; //Just to make things work.

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

struct collisionObject
{
	bool floorCollision = false;
	
	int collidingBody1; //Flat 
	int collidingBody2; //Corner
	Vector collisionNormal;
	int collisionIndexes[4]; //Max of 4 collision indexes - either 1, 2, or 4 collisions
	int numberOfCollidingIndexes = 0;
	
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
		/*
		int collidingBodyIndex;
		int collidingCornerIndex;
		*/
		/*
		int collidingBodyFloorIndex[100]; //Holds lists of bodies that are colliding with floor
		//int unsigned numberOfBodyFloorCollisions = 0; //Tells the resolve collision part how many throught he collidingBodyFloorIndex array to move through.	
		int collidingBFCornerIndex[4]; //Holds list of corners colliding
		//int unsigned collisionBFIndexCounter = 0; //Tells how far to go through collidingBFCornerIndex.
		
		int bodyFloorCollisions[100][4]; //First dimension is the body number. Second dimension is the number of collisions.
		int unsigned numberOfBodyFloorCollisions = 0; //first dimension of array above
		int unsigned collisionBFIndexCounter = 0; //Second dimension of array above
		
		int collidingBody1Index;
		int collidingBody2Index;
		int collidingBB1CornerIndex[4];
		int numOfCollidingBB1 = 0;
		int collidingBB2CornerIndex[4];
		int numOfCollidingBB2 = 0;
		*/
		
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
		
		collisionObject collisionList[maxNumberOfCollisions];
		void createCollisionObject();
		void incrementCollidingIndex(collisionObject &collision);
		void storeCollision(collisionObject Collision, int body1, int body2, Vector collisionNormal, int collisionIndex, bool floorCollision);
		void removeCollisions();
		int getNumberCollidingIndexes();
		void incrementCollision();
		bool isCollisionWithFloor(collisionObject collision);
		
		void Simulate(double DeltaTime);
		void render();
};	

