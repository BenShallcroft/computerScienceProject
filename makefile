semiImplicit: semiImplicit.o
	g++ -o semiImplicit semiImplicit.o -ldrawstuff -L ~/Documents/installs/ode-0.16/drawstuff/src/.libs -lGL -lGLU -lX11

semiImplicit.o: semiImplicit.cc
	g++ -c -I ~/Documents/installs/ode-0.16/include/ semiImplicit.cc
