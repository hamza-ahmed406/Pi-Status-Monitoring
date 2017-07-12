

#ifndef STRFUNC_H
#define STRFUNC_H

#include <stdint.h>
#include <stdbool.h>

int stringLength(char[]);	
char* stringAppend(char* dst, const char* src);
void stringncopy(char *dest, char *src, int Count);
int mstrcmp(const char *s1, const char *s2);

int digit2dec(char hexdigit);
float string2float(char* s);
void num2str(char Str[],uint64_t num);
void float2str(char *str,double floated);
void replaceInArray(char *arr, const char toBeRep, const char replaceWith);
void removeInArray(char *editArr, char *origArr, const char toBeRem);
double truncate(double d);



#endif
