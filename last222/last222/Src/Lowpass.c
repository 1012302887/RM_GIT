#include "main.h"
#define TEST_LENGTH_SAMPLES 320 /* 采样点数 */ 
#define BLOCK_SIZE       32    /* 调用一次arm_fir_f32处理的采样点个数*/ 
#define NUM_TAPS         29    /* 滤波器系数个数*/ 

uint32_t blockSize = BLOCK_SIZE; 
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;   /* 需要调用arm_fir_f32的次数*/ 

static float32_t testInput_f32_50Hz_200Hz[TEST_LENGTH_SAMPLES]; /* 采样点 */ 
static float32_t testOutput[TEST_LENGTH_SAMPLES];      /* 滤波后的输出*/ 
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1]; /* 状态缓存，大小numTaps + blockSize - 1*/ 

/* 低通滤波器系数通过fadtool获取*/ 
const float32_t firCoeffs32LP[NUM_TAPS] = {
   0.001788277179,-0.002203474287, 0.002879403066,-0.003349118168, 0.002751868684,
  -1.177569357e-17,-0.005945665762,  0.01575740241, -0.02946314029,  0.04629049823,
   -0.06468687207,   0.0825278908, -0.09747879207,   0.1074323803,   0.8873986602,
     0.1074323803, -0.09747879207,   0.0825278908, -0.06468687207,  0.04629049823,
   -0.02946314029,  0.01575740241,-0.005945665762,-1.177569357e-17, 0.002751868684,
  -0.003349118168, 0.002879403066,-0.002203474287, 0.001788277179
};
/* 
********************************************************************************************************* 
*    函数名: arm_fir_f32_lp 
*    功能说明: 调用函数arm_fir_f32_lp实现低通滤波器 
*    形  参：无 
*    返 回值: 无 
********************************************************************************************************* 
*/ 
static void arm_fir_f32_lp(void) 
{ 
     uint32_t i; 
     arm_fir_instance_f32 S; 
     float32_t  *inputF32, *outputF32; 

     /* 初始化输入输出缓存指针 */ 
     inputF32 = &testInput_f32_50Hz_200Hz[0]; 
     outputF32 = &testOutput[0]; 

     /* 初始化结构体S */ 
     arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32LP[0], &firStateF32[0], blockSize); 

     /* 实现FIR滤波*/ 
     for(i=0; i < numBlocks; i++) 
     { 
         arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize); 
     } 

     /* 打印滤波后结果*/ 
     for(i=0; i<TEST_LENGTH_SAMPLES; i++) 
     { 
         printf("%f\r\n", testOutput[i]); 
     } 
} 
