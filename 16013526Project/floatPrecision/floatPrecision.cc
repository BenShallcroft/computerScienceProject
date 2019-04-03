//#include "math3d.h"
#include "floatPrecision.h"

#include <drawstuff/drawstuff.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iterator>
#include <assert.h>

using namespace std;
//-----------------------------------------------------------------Global variables

simulation_world *WorldObject;

double mainTIME;

float const epsilon = 0.00001f;

//Next two are for the loading of the cube positions file for the 100 cubes to be spawned
string lineData[100];
int positionArray[100][3];

float bodyPos[3]; //For Drawstuff
float bodyRot[12]; //For Drawstuff
float bodySides[3]; //For Drawstuff

float cubeSize[3] = {0.5, 0.5, 0.5};
float cubePos[3];
int unsigned cubeLoaderCounter = 0;

//Damping - always happens even when not directly used
float const linearDamping = 0.002f;
float const angularDamping = 0.001f;

Vector gravityAccel;

//-----------------------------------------------------------------Load position file functions
void openFile(const char *filename)
{
	ifstream file(filename);
	string str;
	
	int lineCounter = 0;
	while(getline(file, str))
	{
		lineData[lineCounter] = str;
		lineCounter++;
	}
	
	for(int i= 0; i < 100; i++)
	{
		//cout << "LINE " << i+1 << ": " << lineData[i] << endl;
	}
	
	ifstream close();
}

void positionFileToPositions(string dataArray[])
{
	
	for(int i=0; i < 100; i++)
	{
	
		string str = lineData[i];
		istringstream buf(str);
		istream_iterator<string> beg(buf), end;
		
		vector<string> tokens(beg, end);
		
		//cout << "LINE " << i+1 << ": " << tokens[0] << "," << tokens[1] << "," << tokens[2] << endl;
		int xValue = stoi(tokens[0]);
		int yValue = stoi(tokens[1]);
		int zValue = stoi(tokens[2]);
		//cout << "LINE " << i+1 << ": " << xValue << "," << yValue << "," << zValue << endl;
		/*
		xPosArray[i] = xValue;
		yPosArray[i] = yValue;
		zPosArray[i] = zValue;
		*/
		
		positionArray[i][0] = xValue;
		positionArray[i][1] = yValue;
		positionArray[i][2] = zValue;
		
		//cout << "LINE " << i+1 << ": " << positionArray[i][0] << "," << positionArray[i][1] << "," << positionArray[i][2] << endl;
	}
}

//-----------------------------------------------------------------Get pos rot sides function
void getPosRotSides(rigidBody &Body)
{
	
	bodyPos[0] = Body.configState[0].CMposition.x;
	bodyPos[1] = Body.configState[0].CMposition.y;
	bodyPos[2] = Body.configState[0].CMposition.z;
	//cout << "Body pos x,y,z: " << bodyPos[0] << ", " << bodyPos[1] << ", " << bodyPos[2] << endl;
	
	bodySides[0] = Body.width*2;
	bodySides[1] = Body.height*2;
	bodySides[2] = Body.length*2;
	
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
	/*
	cout << "Get pos rot sides function - bodyRot: ";
	for(int i = 0; i < 12; i++)
	{
		cout << bodyRot[i];
		cout << ", ";
	}
	cout << endl;
	*/
}

//-----------------------------------------------------------------Setting up cube position
void cubePositionSetter(int positionArrayIndex)
{
	cubePos[0] = positionArray[positionArrayIndex][0];
	cubePos[1] = positionArray[positionArrayIndex][1];
	cubePos[2] = positionArray[positionArrayIndex][2];
	cout << "Cube pos: ";
	for(int i = 0; i < 3; i++)
	{
		cout << cubePos[i] << ", ";
	}
	cout << endl;
}

//-----------------------------------------------------------------Collision object
void simulation_world::createCollisionObject()
{
	collisionObject &Collision = collisionList[numberOfCollisions];
	
	Collision.numberOfCollidingIndexes = 0;
	
	//numberOfCollisions++;
}

void simulation_world::incrementCollidingIndex(collisionObject &collision)
{
	int placeholder = collision.numberOfCollidingIndexes;
	placeholder++;
	collision.numberOfCollidingIndexes = placeholder;
	cout << "\t\tIncrement colliding index: " << collision.numberOfCollidingIndexes << endl;
}

void simulation_world::storeCollision(collisionObject Collision, int body1, int body2, Vector collisionNormal, int collisionIndex, bool floorCollision)
{
	cout << "-----------------------------" << endl;
	cout << "Added to collision:" << numberOfCollisions << endl;
	
	Collision.collidingBody1 = body1;
	Collision.collidingBody2 = body2;
	cout << "Colliding bodies: " << Collision.collidingBody1 << ", " << Collision.collidingBody2 << endl;

	Collision.collisionNormal = collisionNormal;
	cout << "Collision normal: " << Collision.collisionNormal << endl;
	
	Collision.floorCollision = floorCollision;
	cout << "Is collision with floor? " << Collision.floorCollision << endl;
	
	cout << "Supplied collision index: " << collisionIndex << endl;
	
	//WorldObject->incrementCollidingIndex(Collision);
	//cout << "\t\tNumber of collisions: " << Collision.numberOfCollidingIndexes << endl;
	
	
	//Collision.numberOfCollidingIndexes = 25;
	//Collision.numberOfCollidingIndexes = Collision.numberOfCollidingIndexes + 1;
	
	//Collision.collisionIndexes[Collision.numberOfCollidingIndexes] = collisionIndex;
	
	
		
	Collision.collisionIndexes[Collision.numberOfCollidingIndexes] = collisionIndex;
	cout << "Collision indexes: ";
	for(int i = 0; i < Collision.numberOfCollidingIndexes; i++)
	{
		cout << Collision.collisionIndexes[i] << ", ";
	}
	cout << endl;
	
	cout << "\t\t\tNumber of colliding indexes: " << Collision.numberOfCollidingIndexes << endl;
	Collision.numberOfCollidingIndexes++;
	cout << "\t\t\tNumber of colliding indexes: " << Collision.numberOfCollidingIndexes << endl;
	
	cout << "Before Colliding index master number: " << collidingIndexMasterNumber << endl;
	collidingIndexMasterNumber++;
	cout << "After Colliding index master number: " << collidingIndexMasterNumber << endl;
	cout << "-----------------------------" << endl;
}

void simulation_world::removeCollisions()
{
	cout << "-----------------------------" << endl;
	cout << "Calling remove collisions" << endl;
	collidingIndexMasterNumber = 0;
	numberOfCollisions = 0; //Effectively destroys all collisions.
	cout << "-----------------------------" << endl;
}
int simulation_world::getNumberCollidingIndexes()
{
	cout << "-----------------------------" << endl;
	cout << "Calling get number colliding indexes" << endl;
	cout << "Colliding index master number: " << collidingIndexMasterNumber << endl;
	cout << "-----------------------------" << endl;
	return collidingIndexMasterNumber;
}
void simulation_world::incrementCollision()
{
	cout << "-----------------------------" << endl;
	cout << "Calling increment collision" << endl;
	cout << "Before number of collisions: " << numberOfCollisions << endl;
	numberOfCollisions++;
	cout << "After number of collisions: " << numberOfCollisions << endl;
	cout << "-----------------------------" << endl;
}
bool simulation_world::isCollisionWithFloor(collisionObject collision)
{
	return collision.floorCollision;
}

//-----------------------------------------------------------------Check for collisions
simulation_world::collision_state
simulation_world::checkForCollisions(int configurationIndex)
{
	cout << "Start check for collisions function" << endl;
	collisionState = clear;
	cout << "Collision state start: " << collisionState << endl;
	float const DepthEpsilon = 0.005f;
	
	cout << "Number of bodies: " << numberOfBodies << endl;
	
	//Wide angle check
	//cube check
	//floor check
	
	//Cubes with floor
	for(int BodyIndex = 0; (BodyIndex < numberOfBodies)&&(collisionState != penetrating); BodyIndex++)
	{
		//collisionBFIndexCounter = 0;
		//cout << "Body number: " << BodyIndex << endl;
		cout << "#####Body index: " << BodyIndex << endl;
		rigidBody &Body = BodyList[BodyIndex];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
		
		//Create a collision object to fill if needs to
		WorldObject->createCollisionObject();
		collisionObject &collision = collisionList[numberOfCollisions];
		
		//Used for checking if a collision finished.
		int beforeNumberOfCollidingIndexes = WorldObject->getNumberCollidingIndexes();
		cout << "Before number of colliding indexes: " << beforeNumberOfCollidingIndexes << endl;
		
		//Start Body - Floor collision ------------------------------
		cout << "Body bounding vertexes: " << Body.boundingVertexes << endl;
		for(int unsigned counter = 0; (counter < Body.boundingVertexes) && (collisionState != penetrating); counter++)
		{
			cout << "~~~~~~~" << endl;
			cout << "\t\t\tCheck collisions " << counter << endl;
			cout << "Collision state: " << collisionState << endl;
			Vector position;
			Vector U;
			Vector velocity;
			position = configuration.BodyBoundingVertexes[counter];
			cout << "Vertex position: " << position << endl;
			U = position - configuration.CMposition;
			cout << "Body CM position: " << configuration.CMposition << endl;
			cout << "U = position - CM position: " << U << endl;
			
			cout << "angular velocity: " << configuration.angularVelocity << endl;
			velocity = configuration.CMvelocity + (configuration.angularVelocity % U); //% is cross product
			cout << "Velocity: " << velocity << endl;
			
			cout << "Number of world planes: " << numberOfFloorPlanes << endl;
			
			for(int planeIndex = 0; (planeIndex < numberOfFloorPlanes) && (collisionState != penetrating); planeIndex++)
			{
				
				//planes &plane = worldPlanes[planeIndex];
				floorPlane &Floor = floorList[planeIndex];
				cout << "Floor plane created" << endl;
				
				cout << "Position: " << position << endl;
				cout << "Floor plane normal: " << Floor.planeNormal << endl;
				cout << "Floor d: " << Floor.d << endl;
				float axbyczd = dotProduct(position, Floor.planeNormal) + Floor.d;
				cout << "axbyczd (pos DOT planeNormal + plane.d): " << axbyczd << endl;
				
				if(axbyczd < -DepthEpsilon)
				{
					cout << "axbyczd < - depth epsilon" << endl;
					cout << "\n\n\nBODY COLLISION STATE = PENETRATING\n\n\n" << endl;
					collisionState = penetrating;
				}
				else if(axbyczd < DepthEpsilon)
				{
					cout << "axbyczd < - depth epsilon" << endl;
					//float relativeVelocity = dotProduct(plane.planeNormal, velocity);
					float relativeVelocity = dotProduct(Floor.planeNormal, velocity);
					
					if(relativeVelocity < 0)
					{
						
						cout << "Colliding Objects \n\n\n\n\n\n\n\n\n\n\n\n" << endl;
						collisionState = colliding;
						//collisionNormal = plane.planeNormal;
						collisionNormal = Floor.planeNormal;
						/*
						collidingBFCornerIndex[collisionBFIndexCounter] = counter; 
						cout << "Colliding corner index: " << collidingBFCornerIndex[collisionBFIndexCounter] << endl;
						collidingBodyFloorIndex = BodyIndex;
						cout << "Colliding body index: " << collidingBodyFloorIndex << endl;
						*/
						//bodyFloorCollisions[BodyIndex][collisionBFIndexCounter];
						
						//collisionBFIndexCounter++; 
						//WorldObject -> createCollisionObject();
						cout << "Plane index: "<< planeIndex << endl;
						cout << "Body index: " << BodyIndex << endl;
						cout << "Collision normal: " << collisionNormal <<endl;
						cout << "Collision index: " << counter << endl;
						cout << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#" << endl;
						WorldObject -> storeCollision(collision, planeIndex, BodyIndex, collisionNormal, counter, true);
						cout << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#" << endl;
					}
				}
			}
			
		}
		
		
		int afterNumberOfCollidingIndexes = WorldObject->getNumberCollidingIndexes();
		cout << "After number of colliding indexes: " << afterNumberOfCollidingIndexes << endl;
		if((afterNumberOfCollidingIndexes-beforeNumberOfCollidingIndexes) > 0)
		{
			for(int i = 0; i < (afterNumberOfCollidingIndexes-beforeNumberOfCollidingIndexes); i++)
			{
				WorldObject->incrementCollision();
			}
		}
		else if((afterNumberOfCollidingIndexes-beforeNumberOfCollidingIndexes) <= 0)
		{
			if(numberOfCollisions != 0) numberOfCollisions--;
		}
		
		
		cout << "Number of floor collisions: " << numberOfCollisions << endl;
		//If collision object doesn't have any collisions, delete the object
		
		
		//End Body - Floor collision ------------------------------
		
		//Start Body - Body collision ------------------------------
	}
	
	/*
	cout << "Colliding body - floor indexes: ";
	for(int i = 0; i < collisionBFIndexCounter; i++)
	{
		cout << collidingBFCornerIndex[i] << ", ";
	}
	cout << endl;
	*/
	return collisionState;
}

//-----------------------------------------------------------------Resolve collisions
void simulation_world::resolveCollisions(int configurationIndex)
{
	//Start Body - Floor collision ------------------------------
	//For floor plane/
	cout << "--------------------------------------------------------------------------------" << endl;
	cout << "~~~Start resolve collisions~~~" << endl;
	cout << "--------------------------------------------------------------------------------" << endl;
	
	cout << "Number of floor collisons: " << numberOfCollisions << endl;
	
	for(int unsigned collisionCounter = 0; collisionCounter <= numberOfCollisions; collisionCounter++)
	{
		cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		cout << "Collision resolve number: " << collisionCounter << endl;
		//Loads the collision object
		collisionObject &Collision = collisionList[collisionCounter];
		cout << "Is collision with floor? " << Collision.floorCollision << endl;
		
		//bool collisionWithFloor = Collision.floorCollision;
		//cout << "Collision with floor? " << collisionWithFloor; << endl;
		
		rigidBody &Body = BodyList[Collision.collidingBody2];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
		floorPlane &Plane = floorList[Collision.collidingBody1];
		
		Vector CMvelPlaceholder;
		CMvelPlaceholder.zero();
		Vector angMomPlaceholder;
		angMomPlaceholder.zero();
		
		bool isCollisionWithFloor = Collision.floorCollision;
		cout << "Collision with floor? " << isCollisionWithFloor << endl;
		
		if(!Collision.floorCollision) //With floor collision, only need the floor body and second body
		{
			cout << "Started collisions with floor..." << endl;
		
			
			
			CMvelPlaceholder = configuration.CMvelocity;
			angMomPlaceholder = configuration.angularMomentum;
			
			int numColIndex = Collision.numberOfCollidingIndexes;
			cout << "Number of colliding indexes: " << numColIndex << endl;
			
			//Loops through every colliding index, performs collision response
			for(int unsigned corner = 0; corner < numColIndex; corner++)
			{
				cout << "Colliding corner: " << corner << endl;
				
				Vector positionBody;
				positionBody = configuration.BodyBoundingVertexes[Collision.collisionIndexes[collisionCounter]];
				cout << "Colliding corner position body: " << positionBody << endl;
		
				Vector RBody;
				RBody = positionBody - configuration.CMposition;
				cout << "R - distance to colliding corner from CM position: " << RBody << endl;
	
				Vector velocityBody;
				velocityBody = configuration.CMvelocity + (configuration.angularVelocity % RBody); //Cross product
				cout << "Body Velocity: " << velocityBody << endl;
			
				//Floor stuff
				Vector positionFloor;
				positionFloor.zero();
	
				Vector RFloor;
				RFloor.zero();
	
				Vector velocityFloor;
				velocityFloor.zero();
				cout << "Floor velocity: " << velocityFloor << endl;
	
				Vector velocityDifference;
				velocityDifference = velocityBody - velocityFloor;
				cout << "Velocity difference: " << velocityDifference << endl;
	
				cout << "Collision normal: " << collisionNormal << endl;
				cout << "Coefficient of restitiution: " << Body.coefficientOfRestitution << endl;
		
				//Collision response
				//CH v1
				float impulseNumerator = -(1 + Body.coefficientOfRestitution) * dotProduct(velocityDifference, collisionNormal);
				cout << "Impulse numerator: " << impulseNumerator << endl;
				
				float impulseDenominator = Body.oneOverMass + dotProduct( ( (configuration.oneOverWorldSpaceInertiaTensor*(RBody % collisionNormal) ) % RBody ), collisionNormal );
				cout << "Impulse denominator: " << impulseDenominator << endl;
				
				cout << "Impulse denominator" << impulseDenominator << endl;
				Vector impulse;
				impulse = (impulseNumerator/impulseDenominator) * collisionNormal;
				cout << "Impulse: " << impulse << endl;
				
				//Lovely bit of experimental work here:	
				cout << "-------" << endl;
				cout << "Before CMvelPlaceholder: " << CMvelPlaceholder << endl;	
				CMvelPlaceholder = CMvelPlaceholder + (Body.oneOverMass * impulse);
				cout << "After CMvelPlaceholder: " << CMvelPlaceholder << endl;
		
				cout << "Before angMomPlaceholder: " << angMomPlaceholder << endl;
				angMomPlaceholder = angMomPlaceholder + (RBody % impulse);
				cout << "After angMomPlaceholder: " << angMomPlaceholder << endl;
				cout << "-----------" << endl;
				
				
			}
			
		
		}
		configuration.CMvelocity = CMvelPlaceholder;
		cout << "CM velocity: " << configuration.CMvelocity << endl;
		configuration.angularMomentum = angMomPlaceholder;
		cout << "Body angular momenutm: " << configuration.angularMomentum << endl;
		configuration.angularVelocity = (configuration.oneOverWorldSpaceInertiaTensor * configuration.angularMomentum);
		cout << "Body angular velocity: " << configuration.angularVelocity << endl;
			
		cout << "End floor collisions" << endl;			
		cout << "-------------------" << endl;
		cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		
	}
	WorldObject -> removeCollisions();
	/*
	//cout << "Colliding body index: " << collidingBodyIndex << endl;
	//cout << "Colliding corner index: " << collidingCornerIndex << endl;
	
	Vector CMvelPlaceholder;
	CMvelPlaceholder.zero();
	Vector angMomPlaceholder;
	angMomPlaceholder.zero();
	
	//Number of bodies in the collisions array
	int collisionsArrayBodyNum = sizeof(bodyFloorCollisions)/sizeof(bodyFloorCollisions[0]);
	cout << "Number of colliding bodies: " << collisionsArrayBodyNum << endl;
	
	for(int bodies = 0; bodies < collisionsArrayBodyNum; bodies++)
	{
		rigidBody &Body = BodyList[bodies];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
	
		cout << "Body number: " << bodyFloorCollisions[bodies] << endl;
	
		CMvelPlaceholder = configuration.CMvelocity;
		angMomPlaceholder = configuration.angularMomentum;
		
		for(int unsigned counter = 0; counter < collisionBFIndexCounter; counter++) //Counts through collision indexes
		{
			cout << "Resolve collisions number: " << counter << endl;
		
	
			floorPlane &Floor = floorList[0]; //This needs to be changed in case more floor planes for some reason.
		
			//Body Stuff
			Vector positionBody;
			positionBody = configuration.BodyBoundingVertexes[collidingBFCornerIndex[counter]];
			cout << "Colliding corner position body: " << positionBody << endl;
		
			Vector RBody;
			RBody = positionBody - configuration.CMposition;
			cout << "R - distance to colliding corner from CM position: " << RBody << endl;
	
			Vector velocityBody;
			velocityBody = configuration.CMvelocity + (configuration.angularVelocity % RBody); //Cross product
			cout << "Body Velocity: " << velocityBody << endl;
			
			//Floor stuff
			Vector positionFloor;
			positionFloor.zero();
	
			Vector RFloor;
			RFloor.zero();
	
			Vector velocityFloor;
			velocityFloor.zero();
			cout << "Floor velocity: " << velocityFloor << endl;
	
			Vector velocityDifference;
			velocityDifference = velocityBody - velocityFloor;
			cout << "Velocity difference: " << velocityDifference << endl;
	
	
			cout << "Collision normal: " << collisionNormal << endl;
			cout << "Coefficient of restitiution: " << Body.coefficientOfRestitution << endl;
		
			//Collision response
			//CH v1
			float impulseNumerator = -(1 + Body.coefficientOfRestitution) * dotProduct(velocityDifference, collisionNormal);
		
			//CH v2
			//float impulseNumerator = -dotProduct(collisionNormal, ((1 + Body.coefficientOfRestitution) * velocityDifference));
			cout << "Impulse numerator: " << impulseNumerator << endl;
	
			
			//No dot combine
			float combineInverseMass = Floor.oneOverMass + Body.oneOverMass;
			cout << "Combine inverse mass: " << combineInverseMass << endl;
	
			//Dot combine
			//float dotCombineInverseMass = dotProduct(collisionNormal, (collisionNormal * combineInverseMass));
			//cout << "Combine inverse mass: " << dotCombineInverseMass << endl;
	
	
			float otherDenomStuff = dotProduct(((Floor.oneOverWorldSpaceInertiaTensor*(positionFloor % (positionFloor % collisionNormal))+(configuration.oneOverWorldSpaceInertiaTensor*(positionBody % (positionBody % collisionNormal))))),collisionNormal);
			
	
			float impulseDenominator = Body.oneOverMass + dotProduct( ( (configuration.oneOverWorldSpaceInertiaTensor*(RBody % collisionNormal) ) % RBody ), collisionNormal );
			cout << "Impulse denominator: " << impulseDenominator << endl;
	
		
			//cout << "Other denom stuff: " << otherDenomStuff << endl;
	
			//No dot combine
			//float impulseDenominator = combineInverseMass + otherDenomStuff;
			//Dot combine
			//float impulseDenominator = dotCombineInverseMass + otherDenomStuff;
			cout << "Impulse denominator" << impulseDenominator << endl;
			Vector impulse;
			impulse = (impulseNumerator/impulseDenominator) * collisionNormal;
			cout << "Impulse: " << impulse << endl;
	
	
			/*
			float impulseNumerator = (1 + Body.coefficientOfRestitution) * dotProduct(velocity, collisionNormal);
	
			float impulseDenominator = Body.oneOverMass + dotProduct((configuration.oneOverWorldSpaceInertiaTensor*(multiplyElements(R,collisionNormal))*R), collisionNormal);
	
			Vector impulse;
			impulse = (impulseNumerator/impulseDenominator) * collisionNormal;
			
	
		
			//Lovely bit of experimental work here:	
			cout << "-------" << endl;
			cout << "Before CMvelPlaceholder: " << CMvelPlaceholder << endl;	
			CMvelPlaceholder = CMvelPlaceholder + (Body.oneOverMass * impulse);
			cout << "After CMvelPlaceholder: " << CMvelPlaceholder << endl;
		
			cout << "Before angMomPlaceholder: " << angMomPlaceholder << endl;
			angMomPlaceholder = angMomPlaceholder + (RBody % impulse);
			cout << "After angMomPlaceholder: " << angMomPlaceholder << endl;
			cout << "-----------" << endl;
	
	
	
			//configuration.CMvelocity = configuration.CMvelocity + (Body.oneOverMass * impulse);
			//cout << "CM velocity: " << configuration.CMvelocity << endl;
			//configuration.angularMomentum += (RBody % impulse);
			//cout << "Body angular momenutm: " << configuration.angularMomentum << endl;
	
			//configuration.angularVelocity = (configuration.oneOverWorldSpaceInertiaTensor * configuration.angularMomentum);
			//cout << "Body angular velocity: " << configuration.angularVelocity << endl;
		
			//cout << "End resolve collisions" << endl;
			//cout << "----------------------------------------" << endl;
		}
		configuration.CMvelocity = CMvelPlaceholder;
		cout << "CM velocity: " << configuration.CMvelocity << endl;
		configuration.angularMomentum = angMomPlaceholder;
		cout << "Body angular momenutm: " << configuration.angularMomentum << endl;
		configuration.angularVelocity = (configuration.oneOverWorldSpaceInertiaTensor * configuration.angularMomentum);
		cout << "Body angular velocity: " << configuration.angularVelocity << endl;
	
		cout << "End resolve collisions" << endl;
		cout << "----------------------------------------" << endl;
		
		
		//Start Body - Floor collision ------------------------------
	
	
	}
	*/
	
	
	
	
	
	
	cout << "--------------------------------------------------------------------------------" << endl;
	cout << "~~~End resolve collisions~~~" << endl;
	cout << "--------------------------------------------------------------------------------" << endl;
	
}

//-----------------------------------------------------------------render
void simulation_world::render()
{
	//Test Render
	for(int counter = 0; counter < numberOfBodies; counter++)
	{
		rigidBody &Body = BodyList[counter];
		
		getPosRotSides(Body);
		
		dsDrawBox(bodyPos, bodyRot, bodySides);
	}
	
	/*//TEST DRAW OF A BOX
	float sidesTest[3] = {1.0, 1.0, 1.0};
	float rotTest[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	float posTest[3] = {0.0, 0.0, 1.0};
	
	dsDrawBox(posTest, rotTest, sidesTest);
	*/
}

//-----------------------------------------------------------------Calculate vertices
void simulation_world::calculateVertices(int configurationIndex)
{
	for(int counter = 0; counter < numberOfBodies; counter++)
	{
		rigidBody &Body = BodyList[counter];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
		
		Matrix const &orient = configuration.orientation;
		cout << "Calculate vertices:: orient:\n" << orient << endl;
		Vector const &posit = configuration.CMposition;
		cout << "Calculate vertices:: posit:\n" << posit << endl;
		
		assert(Body.boundingVertexes <= Body.maxBoundingVertexes);
		for(int unsigned i = 0; i < Body.boundingVertexes; i++)
		{
			//cout << "Calculate vertices:: Before Body bounding vertexes: " << configuration.BodyBoundingVertexes[i] << endl;
			configuration.BodyBoundingVertexes[i] = posit + orient * Body.BodyBoundingVertexes[i];
			cout << "Calculate vertices:: After Body bounding vertexes:" << i << ": " << configuration.BodyBoundingVertexes[i] << endl; 
		}
	}
}

//-----------------------------------------------------------------Integrate
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
		
		//cout << "~~~Source, Target configuration indexes: " << SourceConfigIndex << "," << TargetConfigIndex << endl;
		
		//Integrating to get the CM position from the initial position.
		//cout << "\n~~~Start target CM position" << endl;
		//cout << "Source CM vel: " << source.CMvelocity << endl;
		//cout << "Source CM position: " << source.CMposition << endl;

		target.CMposition = source.CMposition + DeltaTime*source.CMvelocity;
		
		cout << "Target CM position: " << target.CMposition << endl;
		
		//Velocity
		//cout << "\n~~~Start target velocity" << endl;
		//cout << "Source CM vel: " << source.CMvelocity << endl;
		//cout << "Delta time: " << DeltaTime << endl;
		//cout << "BodyList[i].oneOverMass" << BodyList[i].oneOverMass << endl;
		//cout << "Source CM force: " << source.CMforce << endl;

		target.CMvelocity = source.CMvelocity + (DeltaTime * BodyList[i].oneOverMass) * source.CMforce;
		
		cout << "Target CM velocity: " << target.CMvelocity << endl;

		//Integrating to get the Orientation of the body
		//target.orientation = source.orientation + tildaMultiply(source.angularVelocity, source.CMposition);
		//cout << "\n~~~Start target orientation" << endl;
		//cout << "Source orientation: " << source.orientation << endl;
		//cout << "Source angularVelocity: " << source.angularVelocity << endl;

		Matrix skewSymmetricAngVel;		
		skewSymmetricAngVel = skewSymmetric(source.angularVelocity);
		//cout << "Skew symmetric angular velocity: " << skewSymmetricAngVel << endl;
		target.orientation = DeltaTime * (skewSymmetricAngVel * source.orientation); //The actual calculation
		cout << "Orientation: " << target.orientation << endl;
		
		//If the orientation comes out as all zero, set orientation as identity matrix
		cout << "Is orientation = zero? " << target.orientation.isZero() << endl;
		if(target.orientation.isZero())
		{
			target.orientation.zero();
			target.orientation.m00 = 1.0;
			target.orientation.m11 = 1.0;
			target.orientation.m22 = 1.0;
		}
		cout << "Orientation: " << target.orientation << endl;
		
		//target.orientation = DeltaTime * tildaMultiply(source.angularVelocity, source.orientation); //This isn't correct - actually Ben, it appears that it is.
		
		//cout << "Target orientation: " << target.orientation << endl;
		
		//Angular momentum
		//cout << "\n~~~Start target angularMomentum" << endl;
		//cout << "Source angular momentum: " << source.angularMomentum << endl;
		//cout << "Delta time: " << DeltaTime << endl;
		//cout << "Source torque: " << source.torque << endl;

		target.angularMomentum = source.angularMomentum + DeltaTime * source.torque;
		
		//cout << "Target angular momentum: " << target.angularMomentum << endl;q

		//Reorthogonalize to remove weird bits
		/*
		int reorthogonalizationRotor = 2;
		int &reorthRef = reorthogonalizationRotor;
		*/
		//cout << "Target orientation: " << target.orientation << endl;
		
		/*
		for(int i = 0; i<=6; i++)
		{
			target.orientation.reorthogonalize(BodyList[i].reorthRef);
		}
		*/
		
		target.orientation.reorthogonalize();
		
		//target.orientation.reorthogonalize(BodyList[i].reorthRef);
		//cout << "\n~~~Reorthogonalized orientation: " << target.orientation << endl;
		
		//inertia tensor
		//cout << "\n~~~Start inverse world space inertia tensor" << endl;
		//cout << "Inverse body space IT: " << BodyList[i].oneOverBodySpaceInertiaTensor << endl;

		target.oneOverWorldSpaceInertiaTensor = target.orientation * BodyList[i].oneOverBodySpaceInertiaTensor * transpose(target.orientation);
		
		//cout << "Target inverse world space IT: " << target.oneOverWorldSpaceInertiaTensor << endl;
		
		//angular velocity
		//cout << "\n~~~Start angular velocity" << endl;
		//cout << "target inverse world space IT: " << target.oneOverWorldSpaceInertiaTensor << endl;
		//cout << "target angular momentum: " << target.angularMomentum << endl;

		target.angularVelocity = target.oneOverWorldSpaceInertiaTensor * target.angularMomentum;
		
		//cout << "Target angular velocity: " << target.angularVelocity << endl;
	}
	cout << "End integrate" << endl;
	cout << "~~~~~~~~~~~~~~~~~\n\n" << endl;
}

//-----------------------------------------------------------------Compute forces
void simulation_world::computeForces(int configurationIndex)
{
	cout << "\n~~~~~~~~~~~~~~~~~" << endl;
	cout << "Started compute Forces" << endl;

	//cout << "configuration index input: " << configurationIndex << endl;
	int i;
	
	for(i = 0; i < numberOfBodies; i++)
	{
		rigidBody &Body = BodyList[i];
		rigidBody::configuration &configuration = Body.configState[configurationIndex];
		
		//TEST
		configuration.torque.zero();
		configuration.CMforce.zero();
		
		//Adding the force that the gravity acceleration imparts on the body
		//configuration.torque += (gravityAccel / Body.oneOverMass);  //Where did this come from?
		configuration.CMforce += gravityAccel / Body.oneOverMass;
		//cout << "Compute forces linear, angular damping: " << -linearDamping << ", " << -angularDamping << endl;
		//cout << "Compute forces CM velocity: " << configuration.CMvelocity << endl;
		//cout << "Compute forces angular velocity: " << configuration.angularVelocity << endl;
		
		//There's always a little damping force on the body even if there's no defined damping
		configuration.CMforce += -linearDamping * configuration.CMvelocity;	//Acts in opposite direction to vertical movement.

		configuration.torque += -angularDamping * configuration.angularVelocity; //Acts in every direction of motion.
		
		cout << "Final compute forces CM force: " << configuration.CMforce << endl;
		cout << "Final compute forces torque: " << configuration.torque << endl;
	}
	cout << "End compute forces" << endl;
	cout << "~~~~~~~~~~~~~~~~~\n" << endl;
}

//-----------------------------------------------------------------Simulate
void simulation_world::Simulate(double DeltaTime)
{
	cout << "Started simulation_world::Simulate" << endl;
	double currentTime = 0.0;
	double targetTime = DeltaTime;
	cout << "Target time: " << targetTime << endl;
	cout << "simulation_world::Simulate DeltaTime: " << DeltaTime << endl;
	
	while(currentTime < DeltaTime)
	{
		cout << "Can confirm, currentTime < DeltaTime" << endl;
		
		computeForces(SourceConfigIndex);
		
		integrate(targetTime - currentTime);
		
		calculateVertices(TargetConfigIndex);
		
		checkForCollisions(TargetConfigIndex);
		
		if(collisionState == penetrating)
		{
			cout << "collision state = penetrating" << endl;
			cout << "Target time: " << targetTime << endl;
			targetTime = (currentTime + targetTime) / 2;
			cout << "Target time: " << targetTime << endl;
			
			//If next bit trips, interpenetration at start frame
			assert(fabs(targetTime - currentTime) > epsilon);
		}
		else
		{
			if(collisionState == colliding)
			{
				int counter = 0;
				do
				{
					resolveCollisions(TargetConfigIndex);
					counter++;
				}
				while((checkForCollisions(TargetConfigIndex) == colliding) && (counter < 100));
				
				assert(counter < 100);
			}
			
		
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

//-----------------------------------------------------------------Simulation loop
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
		if(numberOfBodies < maxNumberOfBodies)
		{
			cubePositionSetter(cubeLoaderCounter);
			WorldObject->buildCubeBody(10.0, cubeSize, cubePos, 1.0);
		}
		WorldObject->Simulate(DeltaTime);
		mainTIME += MaxTimeStep;
	}
	mainTIME = Time;
	WorldObject->render();
	
}

//-----------------------------------------------------------------Build cube body
void simulation_world::buildCubeBody(float Density, float cubeSize[3], float position[3], float coefficientOfRestitution)
{
	if(numberOfBodies < maxNumberOfBodies)
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
		
		Body.coefficientOfRestitution = coefficientOfRestitution;
		cout << "Body coefficient of restitution: " << Body.coefficientOfRestitution << endl;
		
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
		source.orientation.m00 = 1.0;
		source.orientation.m11 = 1.0;
		source.orientation.m22 = 1.0;
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
		//source.angularVelocity.x = 0.9;
		//source.angularVelocity.y = 0.5;
		//source.angularVelocity.z = 0.1;
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
	
	
	//YUCK - unquestionably the worst code I have ever written.
	//Setting up the bounding vertexes of the boxes.
		assert(Body.maxBoundingVertexes >= 8);
		
		Body.boundingVertexes = 8;
	
		Vector boundingVertexVector;
	
		boundingVertexVector.x = width;
		boundingVertexVector.y = height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[0] = boundingVertexVector; //+, +, +
		cout << "Build body:: body bounding vertexes [0]: " << Body.BodyBoundingVertexes[0] << endl;
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[1] = boundingVertexVector; //+, +, -
		cout << "Build body:: body bounding vertexes [1]: " << Body.BodyBoundingVertexes[1] << endl;
	
		boundingVertexVector.y = -height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[2] = boundingVertexVector; //+, -, +
		cout << "Build body:: body bounding vertexes [2]: " << Body.BodyBoundingVertexes[2] << endl;
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[3] = boundingVertexVector; //+, -, -
		cout << "Build body:: body bounding vertexes [3]: " << Body.BodyBoundingVertexes[3] << endl;
	
		boundingVertexVector.x = -width;
		boundingVertexVector.y = height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[4] = boundingVertexVector; //-, +, +
		cout << "Build body:: body bounding vertexes [4]: " << Body.BodyBoundingVertexes[4] << endl;
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[5] = boundingVertexVector; //-, +, -
		cout << "Build body:: body bounding vertexes [5]: " << Body.BodyBoundingVertexes[5] << endl;

		boundingVertexVector.y = -height;
		boundingVertexVector.z = length;
		Body.BodyBoundingVertexes[6] = boundingVertexVector; //-, -, +
		cout << "Build body:: body bounding vertexes [6]: " << Body.BodyBoundingVertexes[6] << endl;
	
		boundingVertexVector.z = -length;
		Body.BodyBoundingVertexes[7] = boundingVertexVector; //-, -, -
		cout << "Build body:: body bounding vertexes [7]: " << Body.BodyBoundingVertexes[7] << endl;
		
		numberOfBodies++;
		cout << "Body built" << endl;
		cout << "~~~~~~~~~~~~~~~~~\n\n" << endl;
	}
	else
	{
		cout << "Maximum number of bodies have been created" << endl;
	}
}

void simulation_world::buildFloor(float Mass)
{
	if(numberOfFloorPlanes < maxNumberOfFloorPlanes)
	{
		floorPlane &Floor = floorList[numberOfFloorPlanes];
		
		Floor.planeNormal.zero();
		Floor.planeNormal.z = 1.0;	//Normal is vertically upwards
		Floor.d = 0.0;
		
		Floor.oneOverMass = 1/Mass;
		cout << "BUILD FLOOR:: inverse mass: " << Floor.oneOverMass << endl;
		
		Floor.centrePosition.zero();
		
		Floor.oneOverBodySpaceInertiaTensor.zero();
		Floor.oneOverBodySpaceInertiaTensor.m00 = 1.0;
		Floor.oneOverBodySpaceInertiaTensor.m11 = 1.0;
		Floor.oneOverBodySpaceInertiaTensor.m22 = 1.0;	//Unit matrix made
		float inertiaMultiplier = Mass * ((Floor.length*Floor.length)/6.0); //The multiplier that goes before the unit matrix in equation before
		Floor.oneOverBodySpaceInertiaTensor = Floor.oneOverBodySpaceInertiaTensor*inertiaMultiplier;
		Floor.oneOverBodySpaceInertiaTensor = inverse(Floor.oneOverBodySpaceInertiaTensor);
		cout << "BUILD FLOOR:: inverse body space IT: " << Floor.oneOverBodySpaceInertiaTensor << endl;
		
		//As the floor is stationary with "infinite" mass and zero velocity change, can calculate the inverse world space inertia tensor now and store it.
		Floor.orientation.zero();
		Floor.orientation.m00 = 1.0;
		Floor.orientation.m11 = 1.0;
		Floor.orientation.m22 = 1.0;
		cout << "BUILD FLOOR:: orientation: " << Floor.orientation << endl;
		
		Floor.oneOverWorldSpaceInertiaTensor = Floor.orientation * Floor.oneOverBodySpaceInertiaTensor * transpose(Floor.orientation);
		cout << "BUILD FLOOR:: inverse world space IT: " << Floor.oneOverWorldSpaceInertiaTensor << endl;
		
		numberOfFloorPlanes++; //Here son
	}
	else
	{
		cout << "Max number of floor planes created" << endl;
	}
}

//-----------------------------------------------------------------Simulation world initialization
simulation_world::simulation_world(float worldX, float worldY, float worldZ) :SourceConfigIndex(0), TargetConfigIndex(1)
{
	buildFloor(50000.0); //Hopefully building the floor plane
	
	//float cubeSize[3] = {0.5, 0.5, 0.5};
	float cubePosTest[3] = {0.0, 0.0, 0.51};		
	buildCubeBody(10.0, cubeSize, cubePosTest, 1.0);
	float cubePosTest2[3] = {3.0, 3.0, 0.6};
	buildCubeBody(10.0, cubeSize, cubePosTest2, 1.0);
	
	calculateVertices(0);
}

simulation_world::~simulation_world()
{
}

//-----------------------------------------------------------------Drawstuff initialization
static void start()
{
	float xyz[3] = {10.0382f,-10.0811f,1.4700f};
	float hpr[3] = {110.0000f,-19.5000f,0.0000f};
	dsSetViewpoint (xyz, hpr);
	
	printf("Simulation start \n");
	
	gravityAccel.z = -9.81;
	WorldObject = new simulation_world(8.0,8.0,8.0);
}
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//Main
int main(int argc, char**argv)
{	
	openFile("cubePositions.txt");		//Opens the text file stored in this directory with 100 x,y,z positions
	positionFileToPositions(lineData);	//The worst named function name ever, but basically changes the array of lines from the text file into a 2D array of cube starting positions ([i][j] where i is a cube, j is the x,y,z positions.)
	
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
	
	dsSimulationLoop(argc, argv, 500, 500, &fn);
	
}	
//-----------------------------------------------------------------
