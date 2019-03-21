semiImplicit: semiImplicit.o
	g++ -o semiImplicit semiImplicit.o -ldrawstuff -L ~/Documents/ODE/ode-0.15.2/drawstuff/src/.libs -lGL -lGLU -lX11

semiImplicit.o: semiImplicit.cc
	g++ -c -I ~/Documents/ODE/ode-0.15.2/include/ semiImplicit.cc
