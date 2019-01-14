#include "main.h"
#define TEST_LENGTH_SAMPLES 320 /* �������� */ 
#define BLOCK_SIZE       32    /* ����һ��arm_fir_f32����Ĳ��������*/ 
#define NUM_TAPS         29    /* �˲���ϵ������*/ 

uint32_t blockSize = BLOCK_SIZE; 
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;   /* ��Ҫ����arm_fir_f32�Ĵ���*/ 

static float32_t testInput_f32_50Hz_200Hz[TEST_LENGTH_SAMPLES]; /* ������ */ 
static float32_t testOutput[TEST_LENGTH_SAMPLES];      /* �˲�������*/ 
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1]; /* ״̬���棬��СnumTaps + blockSize - 1*/ 

/* ��ͨ�˲���ϵ��ͨ��fadtool��ȡ*/ 
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
*    ������: arm_fir_f32_lp 
*    ����˵��: ���ú���arm_fir_f32_lpʵ�ֵ�ͨ�˲��� 
*    ��  �Σ��� 
*    �� ��ֵ: �� 
********************************************************************************************************* 
*/ 
static void arm_fir_f32_lp(void) 
{ 
     uint32_t i; 
     arm_fir_instance_f32 S; 
     float32_t  *inputF32, *outputF32; 

     /* ��ʼ�������������ָ�� */ 
     inputF32 = &testInput_f32_50Hz_200Hz[0]; 
     outputF32 = &testOutput[0]; 

     /* ��ʼ���ṹ��S */ 
     arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32LP[0], &firStateF32[0], blockSize); 

     /* ʵ��FIR�˲�*/ 
     for(i=0; i < numBlocks; i++) 
     { 
         arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize); 
     } 

     /* ��ӡ�˲�����*/ 
     for(i=0; i<TEST_LENGTH_SAMPLES; i++) 
     { 
         printf("%f\r\n", testOutput[i]); 
     } 
} 
