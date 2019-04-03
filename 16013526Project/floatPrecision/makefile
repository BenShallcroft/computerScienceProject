floatPrecisionSimulator: floatPrecision.o
	g++ -o floatPrecisionSimulator floatPrecision.o -ldrawstuff -L ~/Documents/installs/ode-0.16/drawstuff/src/.libs -lGL -lGLU -lX11

floatPrecision.o: floatPrecision.cc
	g++ -c -I ~/Documents/installs/ode-0.16/include/ floatPrecision.cc
