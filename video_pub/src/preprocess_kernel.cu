#include "preprocess_kernel.cuh"

__global__
void cuda_resize(const uchar3* d_rawImg,
                 int rows,int cols,
                 uchar3* d_resizeImg)  //1024*1280 --> 512*640
{
    int2 idx_2d=make_int2((blockIdx.x*blockDim.x)+threadIdx.x,(blockIdx.y*blockDim.y)+threadIdx.y);
    int idx_1d=cols*idx_2d.y+idx_2d.x;
    if(idx_2d.x>=cols||idx_2d.y>=rows)
        return;
    if(idx_2d.x%2==0&&idx_2d.y%2==0)
    {
        int2 res_idx_2d=make_int2(idx_2d.x/2,idx_2d.y/2);
        int res_idx_1d=res_idx_2d.y*cols/2+res_idx_2d.x;

        d_resizeImg[res_idx_1d]=d_rawImg[idx_1d];
    }

}

__global__
void channelComp(const uchar3* input_BGR,
                      int rows,int cols, unsigned char threthold, bool is_tgt_red,
                      unsigned char* redChannel,
                      unsigned char* greenChannel,
                      unsigned char* blueChannel,
                      unsigned char* d_monoImg, // compare result
                      unsigned char* d_compImg //bianry after threth
                      )
{
      int2 idx_2d=make_int2((blockIdx.x*blockDim.x)+threadIdx.x,(blockIdx.y*blockDim.y)+threadIdx.y);
      int idx_1d=cols*idx_2d.y+idx_2d.x;
      if(idx_2d.x>=cols||idx_2d.y>=rows)
          return;
      uchar3 BGR_pix=input_BGR[idx_1d];
      redChannel[idx_1d]=BGR_pix.z;
      greenChannel[idx_1d]=BGR_pix.y;
      blueChannel[idx_1d]=BGR_pix.x;

      //compare red and blue channel
      int comp_pix;
      if(is_tgt_red)        //red-blue
      {
          comp_pix=redChannel[idx_1d]-blueChannel[idx_1d];

      }else
      {
          comp_pix=blueChannel[idx_1d]-redChannel[idx_1d];
      }

      d_monoImg[idx_1d]=comp_pix>0?(unsigned char)comp_pix:0;

      //threath mono to binary
       d_compImg[idx_1d]=(d_monoImg[idx_1d]>threthold)?255:0;



}
void preKernelWrapper(uchar3* d_rawImg, uchar3* d_resizeImg,
                      int rows,int cols, unsigned char threthold, bool is_tgt_red,
                      unsigned char* redChannel,
                      unsigned char* greenChannel,
                      unsigned char* blueChannel,
                      unsigned char* d_monoImg, // compare result
                      unsigned char* d_compImg //bianry after threth
                      )
{
    if(rows==1024&&cols==1280)
    {
        const dim3 blockSize(32,16);
        const dim3 gridSize(1+cols/blockSize.x,1+rows/blockSize.y);
        cuda_resize<<<gridSize,blockSize>>>(d_rawImg,rows,cols,d_resizeImg);

        //update new rows and cols
        rows=512;
        cols=640;
        //update ptr
        d_rawImg=d_resizeImg;
    }

    const dim3 blockSize_split(32,16);
    const dim3 gridSize_split(1+cols/blockSize_split.x,1+rows/blockSize_split.y);

    channelComp<<<gridSize_split,blockSize_split>>>(d_rawImg,
                                                    rows,cols,threthold,is_tgt_red,
                                                    redChannel,
                                                    greenChannel,
                                                    blueChannel,
                                                    d_monoImg,
                                                    d_compImg
                                                    );


}