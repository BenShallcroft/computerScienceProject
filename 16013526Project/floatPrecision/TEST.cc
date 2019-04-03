//#include "math3d.cc"
#include <iostream>
#include <string>
#include <drawstuff/drawstuff.h>

using namespace std;

void simLoop(int pause)
{
	const float sides[3] = {1.0, 1.0, 1.0};
	const float rot[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	
	const float pos1[3] = {0.0, 0.0, 0.5};
	const float pos2[3] = {1.01, 0.6, 0.5};
	const float pos3[3] = {0.4, -0.6, 1.5};
	const float pos4[3] = {1.2, -0.8, 0.5};
	const float pos5[3] = {-0.2, -1.3, 0.5};
	
	dsDrawBox(pos1, rot, sides);
	dsDrawBox(pos2, rot, sides);
	dsDrawBox(pos3, rot, sides);
	dsDrawBox(pos4, rot, sides);
	dsDrawBox(pos5, rot, sides);

}

static void start()
{
	float xyz[3] = {1.0382f,-1.0811f,1.4700f};
	float hpr[3] = {135.0000f,-19.5000f,0.0000f};
	dsSetViewpoint (xyz, hpr);
	
	printf("Simulation start \n");
}

int main(int argc, char**argv)
{
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.stop = 0;
	fn.path_to_textures = "textures";
	
	dsSimulationLoop(argc, argv, 1000, 1000, &fn);
}
