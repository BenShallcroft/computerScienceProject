#include <drawstuff/drawstuff.h>
#include <string>
#include <iostream>
#include <assert.h>

#include "math3d.h"
#include "semiImplicit.h"
//#include <time.h>

//#include <chrono>

using namespace std;
//-----------------------------------------------------------------
//Global variables

simulation_world *WorldObject;

double mainTIME;

float bodyPos[3]; //For Drawstuff
float bodyRot[12]; //For Drawstuff
float bodySides[3]; //For Drawstuff

//Damping - always happens even when not directly used
float const linearDamping = 0.002f;
float const angularDamping = 0.001f;

Vector gravityAccel;

//-----------------------------------------------------------------

//-----------------------------------------------------------------

//Convenience functions
void getPosRotSides(rigidBody &Body)
{
	bodyPos[0] = Body.configState[0].CMposition.x;
	bodyPos[1] = Body.configState[0].CMposition.y;
	bodyPos[2] = Body.configState[0].CMposition.z;
	//cout << "Body pos x,y,z: " << bodyPos[0] << ", " << bodyPos[1] << ", " << bodyPos[2] << endl;
	
	bodySides[0] = Body.width;
	bodySides[1] = Body.height;
	bodySides[2] = Body.length;
	
	/*
	bodyRot[0] = Body.BodyBoundingVertexes[0];
	bodyRot[1] = Body.BodyBoundingVertexes[1];
	bodyRot[2] = Body.BodyBoundingVertexes[2];
	bodyRot[3] = 0.0;
	bodyRot[4] = Body.BodyBoundingVertexes[3];
	bodyRot[5] = Body.BodyBoundingVertexes[4];
	bodyRot[6] = Body.BodyBoundingVertexes[5];
	bodyRot[7] = 0.0;
	bodyRot[8] = Body.BodyBoundingVertexes[6];
	bodyRot[9] = Body.BodyBoundingVertexes[7];
	bodyRot[10] = Body.BodyBoundingVertexes[8];
	bodyRot[11] = 0.0;
	*/
	
	bodyRot[0] = Body.configState[0].orientation.m00;
	bodyRot[1] = Body.configState[0].orientation.m01;
	bodyRot[2] = Body.configState[0].orientation.m02;
	bodyRot[3] = 0.0;
	bodyRot[4] = Body.configState[0].orientation.m10;
	bodyRot[5] = Body.configState[0].orientation.m11;
	bodyRot[6] = Body.configState[0].orientation.m12;
	bodyRot[7] = 0.0;
	bodyRot[8] = Body.configState[0].orientation.m20;
	bodyRot[9] = Body.configState[0].orientation.m21;
	bodyRot[10] = Body.configState[0].orientation.m22;
	bodyRot[11] = 0.0;
//cout << "bodyRot[10] from getPosRotSides method: " << bodyRot[10] << endl;
	
	
}


//-----------------------------------------------------------------

//-----------------------------------------------------------------
//Methods
simulation_world::simulation_world(float worldX, float worldY, float worldZ) :SourceConfigIndex(0), TargetConfigIndex(1)
{
	//buildCubeBody(5.0, 0.5, 0.5, 0.5, 1.0); //Initial body with density 5.0, dimensions of 1 on each edge, coefficient of restitution 1.0.
	calculateVertices(0);
}

simulation_world::~simulation_world()
{
}

void simulation_world::buildCubeBody(float Density, float cubeSize[3], float position[3], float coefficientOfRestitution)
{
	if(numberOfBodies <= maxNumberOfBodies)
	{
cout << "\n \n~~~~~~~~~~~~~~~~~" << endl;
cout << "Body initialising..." << endl;
		rigidBody &Body = BodyList[numberOfBodies];
		
		rigidBody::configuration &source = Body.configState[0];
		rigidBody::configuration &target = Body.configState[1];
		
		source.CMposition.zero();
		source.CMposition.x = position[0];
		source.CMposition.y = position[1];
		source.CMposition.z = position[2];
		
cout << "Body initial CM position: " << source.CMposition << endl;
		
		Body.length = cubeSize[0];
		Body.width = cubeSize[1];
		Body.height = cubeSize[2];
		
		float length = Body.length;
		float width = Body.width;
		float height = Body.height;
cout << "Body length:" << length << endl;
cout << "Body width:" << width << endl;
cout << "Body height:" << height << endl;
	
		float Mass = 8 * Density * cubeSize[0] * cubeSize[1] * cubeSize[2];
cout << "Body mass: " << Mass << endl;
	
		//Body.width =width; //x
		//Body.height = height; //y
		//Body.length = length; //z
		
		target.CMposition.zero();
		source.orientation.zero();
		target.orientation.zero();
		source.CMvelocity.zero();
		target.CMvelocity.zero();
		source.angularMomentum.zero();
		target.angularMomentum.zero();
		source.CMforce.zero();
		target.CMforce.zero();
		source.torque.zero();
		target.torque.zero();
		source.oneOverWorldSpaceInertiaTensor.zero();
		target.oneOverWorldSpaceInertiaTensor.zero();
		source.angularVelocity.zero();
		target.angularVelocity.zero();
	
		Body.oneOverMass = 1.0 / Mass;
cout << "Body inverse mass: " << Body.oneOverMass << endl;
	
		Body.oneOverBodySpaceInertiaTensor.zero();
		
		/*
		Body.oneOverBodySpaceInertiaTensor.m00 = 3/(Mass*(Body.height*Body.height+Body.length*Body.length));
		Body.oneOverBodySpaceInertiaTensor.m11 = 3/(Mass*(Body.width*Body.width+Body.length*Body.length));
		Body.oneOverBodySpaceInertiaTensor.m22 = 3/(Mass*(Body.width*Body.width+Body.height*Body.height));
		*/
		
		
		//I = M * (sides^2 / 6) * (unit matrix)
		Body.oneOverBodySpaceInertiaTensor.m00 = 1.0;
		Body.oneOverBodySpaceInertiaTensor.m11 = 1.0;
		Body.oneOverBodySpaceInertiaTensor.m22 = 1.0;	//Unit matrix made
		
		float inertiaMultiplier = Mass * ((cubeSize[0]*cubeSize[0])/6.0); //The multiplier that goes before the unit matrix in equation before
		Body.oneOverBodySpaceInertiaTensor*inertiaMultiplier;
		Body.oneOverBodySpaceInertiaTensor = inverse(Body.oneOverBodySpaceInertiaTensor);
		
		
cout << "Inverse body space inertia tensor: " << Body.oneOverBodySpaceInertiaTensor << endl;
	
	/*
	//Body.oneOverCMmomentOfInertia = 1/((Mass/12)*(width*width+height*height));
	Body.initial.orientation = 0.0;
	Body.initial.angularVelocity = 0.0;
	Body.initial.torque = 0.0;
	*/
	
	//YUCK - unquestionably the worst code I have ever written.
	//Setting up the bounding vertexes of the boxes.
		assert(Body.maxBoundingVertexes > 8);
		
		Body.boundingVertexes = 8;
	
		Vector boundingVertexVector;
	
		boundingVertexVector.x = width;
		boundingVertexVector.y = height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[0] = boundingVertexVector; //+, +, +
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[1] = boundingVertexVector; //+, +, -
	
		boundingVertexVector.y = -height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[2] = boundingVertexVector; //+, -, +
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[3] = boundingVertexVector; //+, -, -
	
		boundingVertexVector.x = -width;
		boundingVertexVector.y = height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[4] = boundingVertexVector; //-, +, +
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[5] = boundingVertexVector; //-, +, -
	
		boundingVertexVector.y = -height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[6] = boundingVertexVector; //-, -, +
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[7] = boundingVertexVector; //-, -, -
		
		numberOfBodies++;
cout << "Body built" << endl;
cout << "~~~~~~~~~~~~~~~~~\n\n" << endl;
	}
	else
	{
		cout << "Maximum number of bodies have been created" << endl;
	}
}

void simulation_world::Simulate(double DeltaTime)
{
cout << "Started simulation_world::Simulate" << endl;
	double currentTime = 0.0;
	double targetTime = DeltaTime;
cout << "simulation_world::Simulate DeltaTime: " << DeltaTime << endl;
	
	while(currentTime < DeltaTime)
	{
cout << "Can confirm, currentTime < DeltaTime" << endl;
		
		computeForces(SourceConfigIndex);
		
		integrate(targetTime-currentTime);
		
		calculateVertices(TargetConfigIndex);
		/*
		
		
		COLLISION DETECTION WHOOO
		
		
		
		*/
		if(collisionState == penetrating)
		{
		
		}
		else
		{
			currentTime = targetTime;
			targetTime = DeltaTime;
			
			SourceConfigIndex = SourceConfigIndex ? 0 : 1;
			TargetConfigIndex = TargetConfigIndex ? 0 : 1;
cout << "Swapped config indexes" << endl;
cout << "Source index: " << SourceConfigIndex << endl;
cout << "target index: " << TargetConfigIndex << endl;
cout << "\n\n" << endl;
		}
		
	}
}

/*
simulation_world::collision_state
simulation_world::checkForCollisions(int configurationIndex)
{
	collisionState = clear;
	float const DepthEpsilon = 0.001f;
	
	for(int BodyIndex = 0; (BodyIndex < numberOfBodies)&&(collisionState != penetrating); BodyIndex++)
	{
		rigidBody &Body = BodyList[BodyIndex];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
		
		for
	}
}
*/

//Compute forces
void simulation_world::computeForces(int configurationIndex)
{
cout << "\n~~~~~~~~~~~~~~~~~" << endl;
cout << "Started compute Forces" << endl;

cout << "configuration index input: " << configurationIndex << endl;
	int i;
	
	for(i = 0; i < numberOfBodies; i++)
	{
		rigidBody &Body = BodyList[i];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
		
		/*
		//Placeholders for the torque and CMforce the body experiences
		Vector torquePlaceholder;
		torquePlaceholder.zero();
		Vector CMforcePlaceholder;
		torquePlaceholder.zero();
		
		//This effectively means that each iteration of the simulation starts with no force acting on it, it gets added on later:
		configuration.torque = torquePlaceholder;
		configuration.CMforce = CMforcePlaceholder;
		*/
		
		
		//TEST
		configuration.torque.zero();
		configuration.CMforce.zero();
		
		
		//Adding the force that the gravity acceleration imparts on the body
		//configuration.torque += (gravityAccel / Body.oneOverMass);  //Where did this come from?
		configuration.CMforce += gravityAccel / Body.oneOverMass;
cout << "Compute forces linear, angular damping: " << -linearDamping << ", " << -angularDamping << endl;
cout << "Compute forces CM velocity: " << configuration.CMvelocity << endl;
cout << "Compute forces angular velocity: " << configuration.angularVelocity << endl;		
		//There's always a little damping force on the body even if there's no defined damping
		configuration.CMforce += -linearDamping * configuration.CMvelocity;	//Acts in opposite direction to vertical movement.

		configuration.torque += -angularDamping * configuration.angularVelocity; //Acts in every direction of motion.
		
cout << "Final compute forces CM force: " << configuration.CMforce << endl;
cout << "Final compute forces torque: " << configuration.torque << endl;
	}
cout << "End compute forces" << endl;
cout << "~~~~~~~~~~~~~~~~~\n" << endl;
}

//integrate
void simulation_world::integrate(double DeltaTime)
{
cout << "\n~~~~~~~~~~~~~~~~~" << endl;
cout << "Started simulation_world::integrate" << endl;
	int i;

cout << "integrate:: Delta time : " << DeltaTime << endl;

	for(i = 0; i < numberOfBodies; i++)
	{
cout << "\n~~~Body " << i << endl;
		rigidBody::configuration &source = BodyList[i].configState[SourceConfigIndex];
		rigidBody::configuration &target = BodyList[i].configState[TargetConfigIndex];
		
cout << "~~~Source, Target configuration indexes: " << SourceConfigIndex << "," << TargetConfigIndex << endl;
		
		//Integrating to get the CM position from the initial position.
cout << "\n~~~Start target CM position" << endl;
cout << "Source CM vel: " << source.CMvelocity << endl;
cout << "Source CM position: " << source.CMposition << endl;

		target.CMposition = source.CMposition + DeltaTime*source.CMvelocity;
		
cout << "Target CM position: " << target.CMposition << endl;
		
		//Velocity
cout << "\n~~~Start target velocity" << endl;
cout << "Source CM vel: " << source.CMvelocity << endl;
cout << "Delta time: " << DeltaTime << endl;
cout << "BodyList[i].oneOverMass" << BodyList[i].oneOverMass << endl;
cout << "Source CM force: " << source.CMforce << endl;

		target.CMvelocity = source.CMvelocity + (DeltaTime * BodyList[i].oneOverMass) * source.CMforce;
		
cout << "Target CM velocity: " << target.CMvelocity << endl;

		//Integrating to get the Orientation of the body
		//target.orientation = source.orientation + tildaMultiply(source.angularVelocity, source.CMposition);
cout << "\n~~~Start target orientation" << endl;
cout << "Source orientation: " << source.orientation << endl;
cout << "Source angularVelocity: " << source.angularVelocity << endl;

		target.orientation = DeltaTime * tildaMultiply(source.angularVelocity, source.orientation); //This isn't correct - actually Ben, it appears that it is.
		
cout << "Target orientation: " << target.orientation << endl;
		
		//Angular momentum
cout << "\n~~~Start target angularMomentum" << endl;
cout << "Source angular momentum: " << source.angularMomentum << endl;
cout << "Delta time: " << DeltaTime << endl;
cout << "Source torque: " << source.torque << endl;

		target.angularMomentum = source.angularMomentum + DeltaTime * source.torque;
		
cout << "Target angular momentum: " << target.angularMomentum << endl;
		
		int reorthogonalizationRotor = 2;
		int &reorthRef = reorthogonalizationRotor;
		//Reorthogonalize to remove weird bits
		target.orientation.reorthogonalize(reorthRef);
cout << "\n~~~Reorthogonalized orientation: " << target.orientation << endl;
		
		//inertia tensor
cout << "\n~~~Start inverse world space inertia tensor" << endl;
cout << "Inverse body space IT: " << BodyList[i].oneOverBodySpaceInertiaTensor << endl;

		target.oneOverWorldSpaceInertiaTensor = target.orientation * BodyList[i].oneOverBodySpaceInertiaTensor * transpose(target.orientation);
		
cout << "Target inverse world space IT: " << target.oneOverWorldSpaceInertiaTensor << endl;
		
		//angular velocity
cout << "\n~~~Start angular velocity" << endl;
cout << "target inverse world space IT: " << target.oneOverWorldSpaceInertiaTensor << endl;
cout << "target angular momentum: " << target.angularMomentum << endl;

		target.angularVelocity = target.oneOverWorldSpaceInertiaTensor * target.angularMomentum;
		
cout << "Target angular velocity: " << target.angularVelocity << endl;
	}
cout << "End integrate" << endl;
cout << "~~~~~~~~~~~~~~~~~\n\n" << endl;
}


///////////////////////////////////////
void simulation_world::calculateVertices(int configurationIndex)
{
	for(int counter = 0; counter < numberOfBodies; counter++)
	{
		rigidBody &Body = BodyList[counter];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
		
		Matrix const &orient = configuration.orientation;
		Vector const &posit = configuration.CMposition;
		
		assert(Body.boundingVertexes < Body.maxBoundingVertexes);
		for(int unsigned i = 0; i < Body.boundingVertexes; i++)
		{
			configuration.BodyBoundingVertexes[i] = posit + orient * Body.BodyBoundingVertexes[i];
		}
	}
}


void simulation_world::render()
{
	//Test Render
	for(int counter = 0; counter < numberOfBodies; counter++)
	{
		rigidBody &Body = BodyList[counter];
		
		getPosRotSides(Body);
		
		dsDrawBox(bodyPos, bodyRot, bodySides);
	}
	//float sidesTest[3] = {1.0, 1.0, 1.0};
	//float rotTest[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	//float posTest[3] = {1.0, 0.0, 0.0};
	
	//dsDrawBox(posTest, rotTest, sidesTest);
	
	//
}

//Drawstuff sim loop
void simLoop(int pause)
{
cout << "%/%/%/%/%-Start of new SimLoop-%/%/%/%/%" << endl;
	assert(WorldObject);
	//double newTime = time(NULL);
	//cout << "SimLoop new time value: " << newTime << endl;
	//double LastTime = programStartTime - newTime;
	//cout << "SimLoop last time value: " << LastTime << endl;
	
	double const MaxTimeStep = 0.01;
	
	double Time = mainTIME + MaxTimeStep;
cout << "SimLoop: Time " << Time << endl;
	
	while(mainTIME < Time)
	{
		//cout << "can confirm, last time < time" << endl;
		double DeltaTime = Time - mainTIME;
		if(DeltaTime > MaxTimeStep)
		{
cout << "SimLoop: Can confirm, delta time > max time step" << endl;
			DeltaTime = MaxTimeStep;
		}
		WorldObject->Simulate(DeltaTime);
		mainTIME += MaxTimeStep;
	}
	mainTIME = Time;
	WorldObject->render();
	
	/*
	int i;
	for(i = 0; i < numberOfBodies; i++)
	{
		rigidBody &Body = getBodyList(i);
		
		getPosRotSides(Body);
		
		dsDrawBox(bodyPos, bodyRot, bodySides);
	}
	
	//dsDrawBox(pos, rot, sides);
	*/
}

static void start()
{
	float xyz[3] = {1.0382f,-1.0811f,1.4700f};
	float hpr[3] = {135.0000f,-19.5000f,0.0000f};
	dsSetViewpoint (xyz, hpr);
	
	printf("Simulation start \n");
	
	gravityAccel.y = -9.81;
	WorldObject = new simulation_world(100.0, 100.0, 100.0);
	
	float cubeSizeTest[3] = {0.5, 0.5, 0.5};
	float cubePosTest[3] = {0.0, 5.0, 0.0};
	
	WorldObject->buildCubeBody(10.0, cubeSizeTest, cubePosTest, 1.0);
}
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//Main
int main(int argc, char**argv)
{
	//Timer start initial call
	mainTIME = 0.0;
	cout << "Program start time: " << mainTIME << endl;
	
	//--------------------------------
	//drawstuff stuff
	//--------------------------------
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.stop = 0;
	fn.path_to_textures = "textures";
	
	dsSimulationLoop(argc, argv, 1000, 1000, &fn);
	
}	
//-----------------------------------------------------------------
