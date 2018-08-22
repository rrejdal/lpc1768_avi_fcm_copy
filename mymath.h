#ifndef _MYMATH_H_
#define _MYMYTH_H_

#define PI          3.14159265358979f

float SINfD(float a);
float SINfR(float a);
float COSfD(float a);
float COSfR(float a);
float ATAN2fD(float y, float x);
float ATAN2fR(float y, float x);
float ASINfR(float a);

int PRINTd(char *str, int d, int minsize, int leading0, int plus);
int PRINTf(char *str, float f, int minsize, int decimals, int plus);
int PRINTs(char *str, char *instr);
int sPRINTlist(char *out, char *mask, int *d, float *f, int dcount, int fcount);
int sPRINTd(char *out, char *mask, int d1);
int sPRINTdd(char *out, char *mask, int d1, int d2);
int sPRINTddd(char *out, char *mask, int d1, int d2, int d3);
int sPRINTf(char *out, char *mask, float f1);
int sPRINTdf(char *out, char *mask, int d1, float f1);
int sPRINTddf(char *out, char *mask, int d1, int d2, float f1);
int sPRINTfd(char *out, char *mask, float f1, int d1);
int sPRINTff(char *out, char *mask, float f1, float f2);

float power1_7(float value);

#endif
