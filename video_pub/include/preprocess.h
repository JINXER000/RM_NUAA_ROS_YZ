#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <cuda.h>
#include <cuda_runtime.h>
#include<cuda_runtime_api.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
class cuda_proc
{
    cuda_proc(int rows,int cols):_rows(rows),_cols(cols)
    {

    }
    ~cuda_proc();
    void alloc_mem();
    void reAlloc_mem();
    void free_mem();
    void mat2Dmem();
    int _rows,_cols;
}
#endif // PREPROCESS_H
