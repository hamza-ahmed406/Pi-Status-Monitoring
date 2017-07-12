#include "strfunc.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

double truncate(double d){ return (d>0) ? floor(d) : ceil(d) ; }
 
 void float2str(char *str,double floated)
{
	int decimal, truncateed, negative=0;
		int i;
	if (floated<0)
	{
		str[negative++]='-';
		floated*=-1;
	}

	if (floated>=100)
	{
		floated/=100;
		decimal=3;
	}
	else
	{
		floated/=10;
		decimal=2;
	}

	truncateed=truncate(floated);
	for (i=0; i<8; i++)
	{
		if (i==decimal)
		{
			str[i+negative]='.';
			continue;
		}
		else
		{
			str[i+negative]=truncateed+48;
			floated-=truncateed;
			floated*=10;
		truncateed=truncate(floated);
		}
	}
	str[i]='\0';
}

float string2float(char* s) {
	long  integer_part = 0;
	float decimal_part = 0.00;
	float decimal_pivot = 0.10;
	bool isdecimal = false, isnegative = false;
	
	char c;
	while ( ( c = *s++) )  { 
		// skip special/sign chars
		if (c == '-') { isnegative = true; continue; }
		if (c == '+') continue;
		if (c == '.') { isdecimal = true; continue; }
		
		if (!isdecimal) {
			integer_part = (10 * integer_part) + (c - 48);
		}
		else {
			decimal_part += decimal_pivot * (float)(c - 48);
			decimal_pivot /= 10;
		}
	}
	// add integer part
	decimal_part += (float)integer_part;
	
	// check negative
	if (isnegative)  decimal_part = - decimal_part;

	return decimal_part;
}
void num2str(char Str[],uint64_t num)
{
	uint64_t cnt=0;
	uint64_t numDiv=num;
	while (numDiv>=10)
	{
		cnt++;
		numDiv/=10;

	}

	for (int i=cnt; i>=0; i--)
	{
	numDiv=num/pow(10.0,i);
	num-=numDiv*pow(10.0,i);
	Str[cnt-i]=numDiv+48;
	}
}
int mstrcmp(const char *s1, const char *s2)
{
	while((*s1 && *s2) && (*s1 == *s2))
	s1++,s2++;
	return *s1 - *s2;
}

int digit2dec(char digit) {
	if ((int)(digit) >= 65) 
		return (int)(digit) - 55;
	else 
		return (int)(digit) - 48;
}

char* stringAppend(char* dst,const char* src){
	int n,i;
	for(n=0;dst[n]!='\0';n++){
	}
	
	for(i=0;(src[i]!='\0');i++){
		dst[n]=src[i];
		n++;
	}	
	dst[n]='\0';	
	
	return dst;
}


void stringncopy(char *dest, char *src, int Count)
{
	for (int i=0; i<Count; i++)
	{
		dest[i]=src[i];
	}
}


int stringLength(char str[])
{
	int counter;
	for (counter=0; str[counter]!='\0'; counter++){}
	
	return counter;
}

void replaceInArray(char *arr, const char toBeRep, const char replaceWith)
{
	for (int i=0; arr[i]!='\0'; i++)
	{
		if (arr[i] == toBeRep)
			arr[i]= replaceWith;
	}
}

void removeInArray(char *editArr, char *origArr, const char toBeRem)
{
	int u=0;
	for (int i=0; origArr[i]!='\0'; i++)
	{
		if (origArr[i] == toBeRem)
			continue;
		editArr[u++]= origArr[i]; 
	}
}
