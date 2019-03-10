#ifndef FFT_H
#define FFT_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include  <ctime>
#include <iomanip>
#define PI 3.14159265  
#define NI  64       //采样256次

class FFt{
public:
    FFt();
    ~FFt();
public:
typedef struct              //定义结构体
{
    double real;/*实部*/
    double img;/*虚部*/
}complex;

private:
    // float PI = 3.14159265;
    // float N = 64;
public:
complex x[NI * 2], *W;                       /*输出序列的值*/                      //序列长度 全局变量
void add(complex a, complex b, complex *c);
void  sub(complex a, complex b, complex *c) ;
void  mul(complex a, complex b, complex *c) ;
void  divi(complex a, complex b, complex *c) ;
void  initW(int size) ;
void  changex(int size);
void  fftx() ;
void  output();
void save();
};

#endif