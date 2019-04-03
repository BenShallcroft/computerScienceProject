/*
source /opt/cuda6/cuda6.5/cudavars
source /opt/gcc/gccvars-4.8.4.sh
#CARD="-gencode arch=compute_20,code=compute_20" #compatability back to Fermi (GTX 480); optimisation and immediate-launch for none (gives fastest compile times for development)
CARD="-gencode arch=compute_20,code=compute_20 -gencode arch=compute_30,code=compute_30 -gencode arch=compute_30,code=sm_30 -gencode arch=compute_52,code=compute_52 -gencode arch=compute_52,code=sm_52" #compatability back to Fermi (GTX 480); optimisation and immediate-launch for GTX 680 (sm_30) and GTX 980 (sm_52)
nvcc ${CARD} -O3 -ccbin g++ -m64 -std=c++11 -o testGPU testGPU.cu
#./testGPU
qsub testGPU-runs1.sge
*/
/*
source /opt/cuda8/cuda8.0/cudavars
source /opt/gcc/gccvars-4.8.4.sh
#CARD="-gencode arch=compute_20,code=compute_20" #compatability back to Fermi (GTX 480); optimisation and immediate-launch for none (gives fastest compile times for development)
CARD="-gencode arch=compute_20,code=compute_20 -gencode arch=compute_30,code=compute_30 -gencode arch=compute_30,code=sm_30 -gencode arch=compute_52,code=compute_52 -gencode arch=compute_52,code=sm_52 -gencode arch=compute_61,code=compute_61 -gencode arch=compute_61,code=sm_61" #compatability back to Fermi (GTX 480); optimisation and immediate-launch for GTX 680 (sm_30), GTX 980 (sm_52) and GTX 1080 (sm_61)
nvcc ${CARD} -O3 -ccbin g++ -m64 -std=c++11 -o testGPU testGPU.cu
#./testGPU
qsub testGPU-runs1.sge
*/

#include <chrono>
#include <iostream>
#include <iomanip>

const int REP=2500;
const int RUN_TEST=100;
//const int RUN_TIMINGFUNC=10;
const int RUN_TIMINGFUNC=2400; // est. ~4 hours on GTX 980, ~8 hours on GTX 680

void CUDACALL(cudaError err) {
  if (err == cudaSuccess) return;
  std::cerr << "CUDA error: " << cudaGetErrorString(err) << std::endl;
  exit(EXIT_FAILURE);
}

__global__ void mykernel(float * __restrict__ a) {
  int idx = blockIdx.x*blockDim.x + threadIdx.x;
  float r = a[idx];
  #pragma unroll
  for (int n=0;n<REP;n++) r = 0.0001f+r*1.00002f;
  a[idx] = r;
}

void timingfunc(int grid_size, int block_size, float *d_a) {
  float gpu_ms_mean=0, gpu_ms; // time elapsed according to GPU
  float cl1_ms_mean=0; // time elapsed according to CPU-side clock, just (kernel + minimal CPU-side timing code)
  float cl2_ms_mean=0; // time elapsed according to CPU-side clock, all code (will sum to wall clock time)
  cudaEvent_t gpu_before, gpu_after;
  CUDACALL(cudaEventCreate(&gpu_before));
  CUDACALL(cudaEventCreate(&gpu_after));
  for (int run_test=0;run_test<RUN_TEST;run_test++) {
    CUDACALL(cudaEventRecord(gpu_before, 0));
    std::chrono::steady_clock::time_point cl1_before = std::chrono::steady_clock::now();
    static auto cl2_before = cl1_before;
    mykernel <<<grid_size, block_size>>> (d_a);
    CUDACALL(cudaEventRecord(gpu_after, 0));
    CUDACALL(cudaEventSynchronize(gpu_after));
    std::chrono::steady_clock::time_point cl_after = std::chrono::steady_clock::now();
    CUDACALL(cudaEventElapsedTime(&gpu_ms, gpu_before, gpu_after));
    std::chrono::duration<double, std::milli> cl1_time_span = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(cl_after - cl1_before);
    std::chrono::duration<double, std::milli> cl2_time_span = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(cl_after - cl2_before);
    cl2_before=cl_after;
    gpu_ms_mean += gpu_ms/100.0f;
    cl1_ms_mean += cl1_time_span.count()/100.0f;
    cl2_ms_mean += cl2_time_span.count()/100.0f;
  }
  CUDACALL(cudaEventDestroy(gpu_before));
  CUDACALL(cudaEventDestroy(gpu_after));

//std::cout << REP*2 << " FLOP kernel";
//std::cout << "; Mean " << gpu_ms_mean << " ms";
  long FLOPs = block_size*grid_size * (long)REP*2;
  float gpu_GFLOPS = FLOPs / (gpu_ms_mean*1000000);
  float cl1_GFLOPS = FLOPs / (cl1_ms_mean*1000000);
  float cl2_GFLOPS = FLOPs / (cl2_ms_mean*1000000);
  std::cout << std::fixed << std::setprecision(2);
  std::cout << gpu_GFLOPS << " / " << cl1_GFLOPS << " / " << cl2_GFLOPS << " GFLOPS" << std::endl;
}

int main() {
/*
  // GTX 480:
  int grid_size = 15*8;// *32;
  int block_size = 32*2;// *2;
  // GTX 680:
  int grid_size = 8*8;// *32;
  int block_size = 192*2;// *2;
  // GTX 980:
  int grid_size = 16*8;// *32;
  int block_size = 128*2;// *2;
  // GTX 1080:
  int grid_size = 20*8;// *32;
  int block_size = 128*2;// *2;
  // Should be good for all of the above:
  int grid_size = 240*8;// *32;
  int block_size = 384*2;// not *2 as max x-dimension of a block is 1024
*/
  // Should be good for all of the above:
  int grid_size = 240*8*32;
  int block_size = 384*2;// not *2 as max x-dimension of a block is 1024
  long num_bytes = block_size*grid_size * sizeof(float);

  float *h_a = (float*)malloc(num_bytes);
  if (h_a == 0) {std::cerr << "malloc error" << std::endl; exit(EXIT_FAILURE);}
  float *d_a; CUDACALL(cudaMalloc(&d_a, num_bytes));
  CUDACALL(cudaMemset(d_a, 0, num_bytes));

  for (int run_timingfunc=0;run_timingfunc<RUN_TIMINGFUNC;run_timingfunc++) {
    timingfunc(grid_size, block_size, d_a);
  }

  CUDACALL(cudaMemcpy(h_a, d_a, num_bytes, cudaMemcpyDeviceToHost));
  free(h_a);
  CUDACALL(cudaFree(d_a));
  return EXIT_SUCCESS;
}

/* GTX 980 using CUDA 6.5:
[alastair@alastair12 ~]$ nvcc ${CARD} -O3 -ccbin g++ -m64 -std=c++11 -o testGPU testGPU.cu && ./testGPU
4833.60 / 4832.29 / 4832.10 GFLOPS
4833.98 / 4833.70 / 4833.46 GFLOPS
4833.46 / 4833.20 / 4832.97 GFLOPS
4833.51 / 4833.23 / 4833.03 GFLOPS
4833.25 / 4832.99 / 4832.79 GFLOPS
4833.21 / 4832.93 / 4832.73 GFLOPS
4799.90 / 4799.62 / 4799.43 GFLOPS
4766.02 / 4765.74 / 4765.54 GFLOPS
4765.83 / 4765.56 / 4765.36 GFLOPS
4766.19 / 4765.94 / 4765.74 GFLOPS
4764.84 / 4764.56 / 4764.36 GFLOPS
4765.12 / 4764.84 / 4764.63 GFLOPS
4762.39 / 4762.06 / 4761.81 GFLOPS
4764.42 / 4764.12 / 4763.89 GFLOPS
4764.18 / 4763.91 / 4763.69 GFLOPS
4763.87 / 4763.59 / 4763.37 GFLOPS
4764.10 / 4763.83 / 4763.62 GFLOPS
4776.78 / 4776.51 / 4776.32 GFLOPS
4785.27 / 4785.03 / 4784.85 GFLOPS
4786.23 / 4785.98 / 4785.80 GFLOPS
4787.08 / 4786.84 / 4786.66 GFLOPS
4786.52 / 4786.25 / 4786.07 GFLOPS
4786.08 / 4785.84 / 4785.66 GFLOPS
4787.20 / 4786.93 / 4786.73 GFLOPS
4787.47 / 4787.23 / 4787.04 GFLOPS
4786.56 / 4786.31 / 4786.12 GFLOPS
4787.72 / 4787.45 / 4787.27 GFLOPS
4788.01 / 4787.76 / 4787.57 GFLOPS
4788.78 / 4788.50 / 4788.32 GFLOPS
4788.31 / 4788.06 / 4787.88 GFLOPS
4788.25 / 4787.99 / 4787.81 GFLOPS
4788.22 / 4787.97 / 4787.79 GFLOPS
4788.17 / 4787.90 / 4787.72 GFLOPS
4789.13 / 4788.88 / 4788.70 GFLOPS
4788.52 / 4788.26 / 4788.08 GFLOPS
4786.58 / 4786.23 / 4786.01 GFLOPS
4788.91 / 4788.66 / 4788.47 GFLOPS
4788.97 / 4788.73 / 4788.55 GFLOPS
4789.84 / 4789.59 / 4789.41 GFLOPS
4789.99 / 4789.76 / 4789.58 GFLOPS
...
*/

/* My guess is that the higher than expected performance of a GTX 1080 card (below) is due to the use of 
   32-bit floating point multiply units in the Special Function Units (SFUs), in addition to the
   standard functional units ... perhaps giving an extra 4/32 = 12.5% theoretical performance.
*/

/* GTX 1080 using CUDA 6.5 **with CARD not yet having optimisation and immediate-launch for GTX 1080**:
[alastair@alastair12 ~]$ nvcc ${CARD} -O3 -ccbin g++ -m64 -std=c++11 -o testGPU testGPU.cu && ./testGPU
9309.61 / 9305.78 / 9304.97 GFLOPS
9414.90 / 9413.66 / 9412.66 GFLOPS
9424.44 / 9423.22 / 9422.39 GFLOPS
9424.54 / 9423.38 / 9422.56 GFLOPS
9423.89 / 9422.71 / 9421.87 GFLOPS
9413.76 / 9412.58 / 9411.74 GFLOPS
9410.10 / 9408.93 / 9408.08 GFLOPS
9409.76 / 9408.60 / 9407.76 GFLOPS
9410.21 / 9409.05 / 9408.17 GFLOPS
9418.93 / 9417.75 / 9416.91 GFLOPS
9417.13 / 9415.96 / 9415.12 GFLOPS
9418.63 / 9417.44 / 9416.60 GFLOPS
9416.86 / 9415.68 / 9414.81 GFLOPS
9338.27 / 9337.12 / 9336.29 GFLOPS
9338.01 / 9336.84 / 9336.04 GFLOPS
9336.50 / 9335.35 / 9334.54 GFLOPS
9335.29 / 9334.13 / 9333.33 GFLOPS
9344.54 / 9343.36 / 9342.53 GFLOPS
9349.33 / 9348.13 / 9347.29 GFLOPS
9315.99 / 9314.82 / 9314.00 GFLOPS
9291.60 / 9290.42 / 9289.62 GFLOPS
9291.45 / 9290.28 / 9289.47 GFLOPS
9290.18 / 9289.02 / 9288.23 GFLOPS
9289.46 / 9288.27 / 9287.47 GFLOPS
9288.10 / 9286.95 / 9286.15 GFLOPS
9288.66 / 9287.49 / 9286.69 GFLOPS
9290.92 / 9289.74 / 9288.93 GFLOPS
9292.05 / 9290.89 / 9290.08 GFLOPS
9280.50 / 9279.33 / 9278.53 GFLOPS
9239.80 / 9238.64 / 9237.85 GFLOPS
9238.17 / 9237.01 / 9236.22 GFLOPS
9238.42 / 9237.28 / 9236.50 GFLOPS
9239.73 / 9238.55 / 9237.73 GFLOPS
9238.66 / 9237.49 / 9236.68 GFLOPS
9238.32 / 9237.13 / 9236.34 GFLOPS
9238.38 / 9237.22 / 9236.43 GFLOPS
9239.39 / 9238.24 / 9237.43 GFLOPS
9239.75 / 9238.57 / 9237.78 GFLOPS
9239.99 / 9238.80 / 9238.01 GFLOPS
9239.70 / 9238.52 / 9237.75 GFLOPS
...
9109.84 / 9108.70 / 9107.91 GFLOPS
9111.16 / 9109.99 / 9109.19 GFLOPS
9110.61 / 9109.46 / 9108.68 GFLOPS
9113.57 / 9112.40 / 9111.60 GFLOPS
9110.60 / 9109.46 / 9108.68 GFLOPS
9111.35 / 9110.20 / 9109.43 GFLOPS
9112.75 / 9111.61 / 9110.85 GFLOPS
9110.69 / 9109.56 / 9108.79 GFLOPS
9111.97 / 9110.86 / 9110.09 GFLOPS
9112.12 / 9111.00 / 9110.25 GFLOPS
9110.65 / 9109.53 / 9108.77 GFLOPS
9110.62 / 9109.49 / 9108.74 GFLOPS
9111.94 / 9110.83 / 9110.08 GFLOPS
9111.87 / 9110.75 / 9109.99 GFLOPS
9109.50 / 9108.39 / 9107.64 GFLOPS
9110.69 / 9109.56 / 9108.80 GFLOPS
9113.08 / 9111.95 / 9111.14 GFLOPS
9111.69 / 9110.57 / 9109.74 GFLOPS
9111.86 / 9110.74 / 9109.93 GFLOPS
9111.63 / 9110.52 / 9109.76 GFLOPS
9107.44 / 9106.36 / 9105.58 GFLOPS
9113.04 / 9111.94 / 9111.13 GFLOPS
9112.03 / 9110.93 / 9110.12 GFLOPS
9109.63 / 9108.52 / 9107.71 GFLOPS
9112.84 / 9111.76 / 9110.99 GFLOPS
9111.47 / 9110.40 / 9109.61 GFLOPS
9111.63 / 9110.55 / 9109.77 GFLOPS
9110.70 / 9109.60 / 9108.81 GFLOPS
9111.91 / 9110.82 / 9110.04 GFLOPS
9110.48 / 9109.38 / 9108.57 GFLOPS
9110.78 / 9109.71 / 9108.93 GFLOPS
9110.69 / 9109.61 / 9108.84 GFLOPS
9111.01 / 9109.95 / 9109.18 GFLOPS
9110.60 / 9109.53 / 9108.76 GFLOPS
9110.37 / 9109.29 / 9108.51 GFLOPS
9110.81 / 9109.72 / 9108.88 GFLOPS
9110.50 / 9109.42 / 9108.62 GFLOPS
9111.17 / 9110.08 / 9109.28 GFLOPS
9110.49 / 9109.43 / 9108.66 GFLOPS
9110.89 / 9109.83 / 9109.07 GFLOPS
...
*/

/* GTX 1080 using CUDA 8.0:
[alastair@alastair12 ~]$ nvcc ${CARD} -O3 -ccbin g++ -m64 -std=c++11 -o testGPU testGPU.cu && ./testGPU
nvcc warning : The 'compute_20', 'sm_20', and 'sm_21' architectures are deprecated, and may be removed in a future release (Use -Wno-deprecated-gpu-targets to suppress warning).
9336.36 / 9332.39 / 9330.98 GFLOPS
9497.25 / 9495.93 / 9494.24 GFLOPS
9486.74 / 9485.43 / 9483.89 GFLOPS
9483.04 / 9481.70 / 9480.08 GFLOPS
9481.02 / 9479.70 / 9478.17 GFLOPS
9480.61 / 9479.31 / 9477.82 GFLOPS
9488.93 / 9487.62 / 9486.10 GFLOPS
9490.42 / 9489.09 / 9487.56 GFLOPS
9490.31 / 9488.97 / 9487.47 GFLOPS
9482.12 / 9480.78 / 9479.25 GFLOPS
9426.74 / 9425.43 / 9423.91 GFLOPS
9427.52 / 9426.19 / 9424.70 GFLOPS
9426.72 / 9425.43 / 9423.94 GFLOPS
9427.51 / 9426.21 / 9424.72 GFLOPS
9427.24 / 9425.92 / 9424.42 GFLOPS
9353.34 / 9352.06 / 9350.53 GFLOPS
9353.25 / 9351.96 / 9350.46 GFLOPS
9357.97 / 9356.67 / 9355.16 GFLOPS
9362.79 / 9361.47 / 9359.94 GFLOPS
9362.92 / 9361.63 / 9360.10 GFLOPS
9361.30 / 9359.98 / 9358.48 GFLOPS
9362.52 / 9361.25 / 9359.79 GFLOPS
9361.41 / 9360.12 / 9358.65 GFLOPS
9361.92 / 9360.60 / 9359.13 GFLOPS
9342.27 / 9340.96 / 9339.49 GFLOPS
9310.85 / 9309.59 / 9308.18 GFLOPS
9310.07 / 9308.81 / 9307.34 GFLOPS
9310.45 / 9309.15 / 9307.62 GFLOPS
9310.14 / 9308.87 / 9307.40 GFLOPS
9310.52 / 9309.23 / 9307.74 GFLOPS
9310.50 / 9309.20 / 9307.70 GFLOPS
9310.21 / 9308.92 / 9307.39 GFLOPS
9309.94 / 9308.67 / 9307.17 GFLOPS
9309.05 / 9307.80 / 9306.35 GFLOPS
9310.60 / 9309.33 / 9307.87 GFLOPS
9309.92 / 9308.65 / 9307.22 GFLOPS
9310.48 / 9309.20 / 9307.78 GFLOPS
9310.28 / 9308.99 / 9307.54 GFLOPS
9310.74 / 9309.46 / 9308.01 GFLOPS
9290.50 / 9289.25 / 9287.98 GFLOPS
...
9177.33 / 9176.24 / 9175.48 GFLOPS
9176.52 / 9175.43 / 9174.68 GFLOPS
9176.80 / 9175.72 / 9174.97 GFLOPS
9177.21 / 9176.13 / 9175.37 GFLOPS
9175.25 / 9174.17 / 9173.41 GFLOPS
9176.36 / 9175.26 / 9174.50 GFLOPS
9177.45 / 9176.36 / 9175.60 GFLOPS
9177.45 / 9176.37 / 9175.62 GFLOPS
9177.29 / 9176.18 / 9175.42 GFLOPS
9177.55 / 9176.46 / 9175.71 GFLOPS
9176.14 / 9175.05 / 9174.31 GFLOPS
9176.54 / 9175.47 / 9174.71 GFLOPS
9177.17 / 9176.09 / 9175.33 GFLOPS
9176.00 / 9174.93 / 9174.17 GFLOPS
9176.30 / 9175.21 / 9174.46 GFLOPS
9175.58 / 9174.49 / 9173.72 GFLOPS
9176.64 / 9175.57 / 9174.81 GFLOPS
9176.32 / 9175.23 / 9174.48 GFLOPS
9175.38 / 9174.29 / 9173.51 GFLOPS
9176.35 / 9175.29 / 9174.52 GFLOPS
9176.89 / 9175.81 / 9175.06 GFLOPS
9175.53 / 9174.44 / 9173.64 GFLOPS
9175.92 / 9174.83 / 9174.04 GFLOPS
9174.73 / 9173.66 / 9172.89 GFLOPS
9162.75 / 9161.66 / 9160.90 GFLOPS
9117.35 / 9116.28 / 9115.52 GFLOPS
9116.27 / 9115.19 / 9114.45 GFLOPS
9116.67 / 9115.58 / 9114.84 GFLOPS
9116.94 / 9115.83 / 9115.00 GFLOPS
9116.89 / 9115.81 / 9115.01 GFLOPS
9118.10 / 9117.02 / 9116.27 GFLOPS
9117.04 / 9115.97 / 9115.22 GFLOPS
9116.26 / 9115.19 / 9114.42 GFLOPS
9116.97 / 9115.91 / 9115.16 GFLOPS
9116.97 / 9115.88 / 9115.11 GFLOPS
9116.28 / 9115.20 / 9114.43 GFLOPS
9117.29 / 9116.23 / 9115.49 GFLOPS
9116.72 / 9115.65 / 9114.90 GFLOPS
9115.79 / 9114.71 / 9113.96 GFLOPS
9117.05 / 9115.97 / 9115.22 GFLOPS
...
*/
