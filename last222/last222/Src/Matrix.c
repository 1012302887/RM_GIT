//==============       该文件矩阵均采用一维数组, 以提高效率      =============//
/* Includes ------------------------------------------------------------------*/
#include "main.h"	
#include "kalman.h"	
#include "matrix.h"	
#include <math.h>		


/*单独使用需调用的头文件
#include  "matrix.h"
#include  "stdlib.h"
#include  "math.h"
*/


//============================================================================//
//==                          实矩阵相加                                    ==//
//============================================================================//
//==函数说明: m*n阶矩阵和m*n阶矩阵相加, 矩阵均以行优先存储                  ==//
//==入口参数: *a                指向左矩阵的指针                            ==//
//==          *b                指向右矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixAdd(float *a, float *b, float *c, unsigned char m, unsigned char n)
{
  unsigned char i;
  
  for (i=0; i<m*n; i++)
  {
    c[i] = a[i] + b[i];
  }
}





//============================================================================//
//==                          实矩阵相减                                    ==//
//============================================================================//
//==函数说明: m*n阶矩阵和m*n阶矩阵相减, 矩阵均以行优先存储                  ==//
//==入口参数: *a                指向左矩阵的指针                            ==//
//==          *b                指向右矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixMinus(float *a, float *b, float *c, unsigned char m, unsigned char n)
{
  unsigned char i;
  
  for (i=0; i<m*n; i++)
  {
    c[i] = a[i] - b[i];
  }
}





//============================================================================//
//==                          实矩阵相乘                                    ==//
//============================================================================//
//==函数说明: m*p阶矩阵和p*n阶矩阵相乘, 矩阵均以行优先存储                  ==//
//==入口参数: *a                指向左矩阵的指针                            ==//
//==          *b                指向右矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 左矩阵行数                                  ==//
//==          p                 左矩阵列数(右矩阵行数)                      ==//
//==          n                 右矩阵列数                                  ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixMul(float *a, float *b, float *c, unsigned char m, unsigned char p, unsigned char n)
{
  unsigned char i,j,k;                      //循环变量
  
  for (i=0; i<m; i++)                       //扫描矩阵的行
  {
    for (j=0; j<n; j++)                     //扫描矩阵的列
    {
      c[i*n+j] = 0.0;
      for (k=0; k<p; k++)
      {
                                            //按照矩阵乘法的概念计算
        c[i*n+j] += a[i*p+k] * b[k*n+j];    //a[i*p+k]: 由于k的循环,扫描了a[]数组的第i行
                                            //b[k*n+j]: 由于k的循环,扫描了a[]数组的第j行
      }
    }
  }
}





//============================================================================//
//==                            实矩阵求转置                                ==//
//============================================================================//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixTrans(float *a, float *c, unsigned char m, unsigned char n)
{
  unsigned char i,j;
  
  for (i=0; i<n; i++)
  {
    for (j=0; j<m; j++)
    {
      c[i*m+j] = a[j*n+i];      //对于m*n的矩阵,i*m+j表示i行j列的元素; 对于n*m的矩阵,i*n+j表示i行j列的元素
    }
  }
}





//============================================================================//
//==                          实矩阵求行列式1                               ==//
//============================================================================//
//==函数说明: 根据行列式展开法求得, 对高阶行列式有问题                      ==//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: 无                                                            ==//
//==返回值:   X                 行列式的值                                  ==//
//============================================================================//
float MatrixDet1(float *a, unsigned char m, unsigned char n)
{
  signed char i, j, k, p, r;                  //循环变量
  float Temp=1, Temp1=1, S=0, S1=0;           //辅助变量
  float X;                                    //行列式的值
  
  if (n==2)
  {
    for(i=0; i<m; i++)                        //扫描矩阵的行
    {
      for(j=0 ;j<n; j++)                      //扫描矩阵的列
      {
        if((i+j)%2)                           //根据行优先的法则,对角线元素下标之和为偶数
        {
          Temp1 *= a[i*n+j];
        }
        else
        {
          Temp  *= a[i*n+j];
        }
      }
    }
    X=Temp-Temp1;
  }
  else
  {
    for (k=0; k<n; k++)                       //计算行列式展开式中的被减数
    { 
      for (i=0,j=k; i<m&&j<n; i++,j++)        //斜线方向的数据相乘
      {
        Temp *= a[i*n+j];
      }
      if (m-i)                                //每次相乘都是m个元素,若不足m个元素,则需要继续处理
      { 
        for (p=m-i,r=m-1; p>0; p--,r--)
        {
          Temp  *= a[r*n+p-1];
        }
      }
      S += Temp;
      Temp = 1;
    }
      
      
    for (k=n-1; k>=0; k--)                    //计算行列式展开式中的减数
    { 
      for(i=0,j=k; i<m&&j>=0; i++,j--)
      {
        Temp1 *= a[i*n+j];
      }
      if (m-i) 
      {
        for(p=m-1,r=i; r<m; p--,r++)
        {
          Temp1 *= a[r*n+p];
        }
      } 
      S1 += Temp1;
      Temp1 = 1;
    }
    X=S-S1;
  }
  
  return   X;
}





//============================================================================//
//==                           实矩阵求逆1                                  ==//
//============================================================================//
//==函数说明: 根据矩阵行列式等求实矩阵的逆矩阵,对高阶行列式有问题           ==//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          *c                指向逆矩阵的指针                            ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向逆矩阵的指针                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixInv1(float *a, float *c, unsigned char m, unsigned char n)
{
  unsigned char i,j,k,x,y;
  float AB[LENGTH], SP[LENGTH], B[LENGTH];
  float X;                                    //行列式的值
  
  X = MatrixDet1(a, m, n);
  X = 1/X;                                    //行列式的倒数
  
  for (i=0; i<m; i++)
  {
    for (j=0; j<n; j++)
    {
      for (k=0; k<m*n; k++)
      {
        B[k] = a[k];                          //初始化辅助矩阵B=a
      }
      for(x=0; x<n; x++)
      {
        B[i*n+x] = 0;                         //i行所有元素清零
      }
      for(y=0; y<m; y++)
      {
        B[m*y+j] = 0;                         //j列所有元素清零
      }
      B[i*n+j] = 1;                           //当前的主元置1
      SP[i*n+j] = MatrixDet1(B,m,n);
      AB[i*n+j] = X*SP[i*n+j];
    }
  }
  MatrixTrans(AB, c, m, n);
}





//============================================================================//
//==                     Gauss-Jordan法实矩阵求逆                           ==//
//============================================================================//
//==函数说明: 求n阶矩阵的逆矩阵,矩阵以行优先存储; 结果保存在源数组里,因此需 ==//
//==          注意会破坏源数组的数据;                                       ==//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          n                 矩阵阶数                                    ==//
//==出口参数: *a                指向逆矩阵的指针                            ==//
//==返回值:   BOOL              操作成功、失败                              ==//
//============================================================================//
unsigned char Gauss_Jordan(float *a, unsigned char n)
{
    signed char i,j,k,l,u,v;
    signed char is[ORDER];
    signed char js[ORDER];
    float d,p;
    
    for (k=0; k<=n-1; k++)
      { d=0.0;
        for (i=k; i<=n-1; i++)
        for (j=k; j<=n-1; j++)
          { l=i*n+j; p=fabs(a[l]);
            if (p>d) { d=p; is[k]=i; js[k]=j;}
          }
        if (d+1.0f==1.0f)
          {
            return(0);
          }
        if (is[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=is[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (js[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+js[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        l=k*n+k;
        a[l]=1.0f/a[l];
        for (j=0; j<=n-1; j++)
          if (j!=k)
            { u=k*n+j; a[u]=a[u]*a[l];}
        for (i=0; i<=n-1; i++)
          if (i!=k)
            for (j=0; j<=n-1; j++)
              if (j!=k)
                { u=i*n+j;
                  a[u]=a[u]-a[i*n+k]*a[k*n+j];
                }
        for (i=0; i<=n-1; i++)
          if (i!=k)
            { u=i*n+k; a[u]=-a[u]*a[l];}
      }
    for (k=n-1; k>=0; k--)
      { if (js[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=js[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (is[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+is[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
      }
    
    return(1);
}





//============================================================================//
//==                            计算A*B*A'                                  ==//
//============================================================================//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          *b                指向实矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          n                 矩阵的阶数                                  ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixCal(float *a, float *b, float *c, unsigned char n)
{
  float Temp1[LENGTH] = {0};
  float Temp2[LENGTH] = {0};
  
  MatrixMul(a, b, Temp1, n, n, n);
  MatrixTrans(a, Temp2, n, n);
  MatrixMul(Temp1, Temp2, c, n, n, n);
}
