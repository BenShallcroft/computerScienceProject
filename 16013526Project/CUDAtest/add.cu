

//In theory, GPU accelerated code


#include <iostream>
#include <math.h>

using namespace std;

__global__ //Kernel function to add the elements of two arrays
void add(int n, float *x, float *y)
{	
	for(int i= 0; i < n; i++) y[i] = x[i] + y[i]; //Note: i is now the thread index, and each loop through changes to next thread in the block
}

int main(void)
{
	int N = 1<<20;
	
	float *x, *y;
	
	//Allovate unified memory - accessible from CPU or GPU
	cudaMallocManaged(&x, N*sizeof(float));
	cudaMallocManaged(&y, N*sizeof(float));
	
	//Initiallise the x and y arrays on the host
	for(int i= 0; i < N; i++)
	{
		x[i] = 1.0f;
		y[i] = 2.0f;
	}
	
	//Run kernel on 1M elements on the GPU
	add<<<1, 1>>>(N, x, y);
	
	//Wait for GPU to finish before accessing on host
	cudaDeviceSynchronize();
	
	float maxError = 0.0f;
	
	for(int i=0; i < N; i++) maxError = fmax(maxError, fabs(y[i] -3.0f));
	
	cout << "Max error: " << maxError << endl;
	
	//Free memory
	cudaFree(x);
	cudaFree(y);
	
	return 0;
}
