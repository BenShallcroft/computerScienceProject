#include "math3d.h"

int numberOfBodies = 0;
const int maxNumberOfBodies = 10;
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
	enum{maxBoundingVertexes = 20};
	Vector BodyBoundingVertexes[maxBoundingVertexes];
	
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

//-------------------------------------------------------------------------------------------------------------------------

//simulation_world object -> object for the world.
class simulation_world
{
	private:		
		float worldX, worldY, worldZ;
		
		//Collision part
		enum collision_state
		{
			penetrating, colliding, clear //If bodies are inside each other, just colliding, or not colliding at all
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
		
		int SourceConfigIndex;
		int TargetConfigIndex;
		
	public:
		simulation_world(float worldX, float worldY, float worldZ); //constructor
		~simulation_world(); //Destructor
		
		rigidBody BodyList[maxNumberOfBodies];
		void buildCubeBody(float Density, float cubeSize[3], float position[3], float coefficientOfRestitution);
		
		void Simulate(double DeltaTime);
		void render();
		//int getNumOfBodies(){return numberOfBodies;}	
};
