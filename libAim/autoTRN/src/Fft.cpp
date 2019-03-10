#include "Fft.h"
using namespace std;
 

FFt::FFt(){}


FFt::~FFt(){}

void FFt::add(complex a, complex b, complex *c)      //  复数加运算
{
    c->real = a.real + b.real;
    c->img = a.img + b.img;
}
void  FFt::sub(complex a, complex b, complex *c)      //  复数减运算
{
    c->real = a.real - b.real;
    c->img = a.img - b.img;
}
void  FFt::mul(complex a, complex b, complex *c)      //复数乘运算
{
    c->real = a.real*b.real - a.img*b.img;
    c->img = a.real*b.img + a.img*b.real;
}
void  FFt::divi(complex a, complex b, complex *c)      //  复数除运算
{
    c->real = (a.real*b.real + a.img*b.img) / (b.real*b.real + b.img*b.img);
    c->img = (a.img*b.real - a.real*b.img) / (b.real*b.real + b.img*b.img);
}
 
void  FFt::initW(int size)                                //欧拉公式运算   
{
    int i;
    W = (complex*)malloc(sizeof(complex)* size);    //分配内存空间
    for (i = 0; i<size; i++)
    {
        W[i].real = cos(2 * PI / size*i);
        W[i].img = -1 * sin(2 * PI / size*i);
    }
    /*for (i = 0; i < size; i++)
    {
    cout << endl;
    cout << endl;
    cout << endl;
    cout << W[i].real << " ";
    cout << W[i].img <<"j" << " ";
    }*/
}
 
void  FFt::changex(int size)                      //变址运算  
{
    complex temp;
    unsigned int i = 0, j = 0, k = 0;
    double t;
    for (i = 0; i<size; i++)
    {
        k = i; j = 0;
        t = (log(size) / log(2));
        while ((t--)>0)
        {
            j = j << 1;
            j |= (k & 1);
            k = k >> 1;
        }
        if (j>i)
        {
            temp = x[i];
            x[i] = x[j];
            x[j] = temp;
        }
    }
}
 
 
void  FFt::fftx()                             //快速傅里叶函数
{
    long    int i = 0, j = 0, k = 0, l = 0;
    complex up, down, product;
    changex(NI);
    for (i = 0; i<log(NI) / log(2); i++) /*一级蝶形运算*/
    {
        l = 1 << i;
        for (j = 0; j<NI; j += 2 * l) /*一组蝶形运算*/
        {
            for (k = 0; k<l; k++) /*一个蝶形运算*/
            {
                mul(x[j + k + l], W[NI*k / 2 / l], &product);
                add(x[j + k], product, &up);
                sub(x[j + k], product, &down);
                x[j + k] = up;
                x[j + k + l] = down;
            }
        }
    }
}
 
 
void  FFt::output()   /*输出x结果*/
{
    int i;
    printf("\nx傅里叶变换结果\n");
    for (i = 0; i<NI; i++)
    {
        if (i % 4 == 0 && i != 0) printf("\n");
        printf("  %.2f", x[i].real);
        if (x[i].img >= 0.0001)
            printf("+%.2fj  ", x[i].img);
        else if (fabs(x[i].img)<0.0001)
            printf("+0.0000j  ");
        else
            printf("%.2fj  ", x[i].img);
    }
    printf("\n");
}
 
void FFt::save()           //保存x傅里叶变换结果
{
    int i;
    ofstream outfile("/home/weepies/output.txt", ios::out);
    if (!outfile)
    {
        cerr << "open result1.txt erro!" << endl;
        exit(1);
    }
    outfile << "x傅里叶变换结果:" << endl;
    for (i = 0; i<NI; i++)
    {
        outfile << x[i].real;
        if (x[i].img >= 0.0001)
            outfile << "+" << x[i].img << "j" << " ";
        else   if (fabs(x[i].img)<0.0001)
            outfile << "+" << "0.00" << "j" << " ";
        else
            outfile << x[i].img << "j" << " ";
    }
    printf("\n");
    outfile.close();
}