#include <stdio.h>
#include <math.h>
#include <string.h>
#include "bernstin.h"


/*
  ŒÆËãnŽÎ¶àÏîÊœµÄÏµÊý
  n   ŽÎÊý
  u   [0-1]
  B[i] ÏµÊýÏî
*/

void AllBernstin(int n, float u, float* B)
{
	int j,k;
	float u1,saved,temp;
	B[0] = 1.0;
	u1 = 1 - u;
	for (j = 1; j <= n; j++)
	{
		saved = 0.0;

		for (k = 0; k < j; k++)
		{
			temp = B[k];
			B[k] = saved + u1*temp;
			saved = u*temp;
		}
		B[j] = saved;
	}
}

/*
 ŒÆËãÔÚBezierCurveÉÏµÄµã
*/
void PointOnBezierCurve(int n, float u, float* Px, float* Py,float* C)
{
	int k;
	float B[5];
	AllBernstin(n, u, B);
	C[0] = 0;
	C[1] = 0;
	for (k = 0; k <= n; k++)
	{
		C[0] = C[0] + B[k] * Px[k];
		C[1] = C[1] + B[k] * Py[k];
	}
}


/*
  ŒÆËãBezierCeuveÉÏµÄµãµÄÇúÂÊ
  return ÇúÂÊ
*/
float CurvatureOnBezierCeuve(int n,float u,float* Px,float* Py)
{
	int i;
	float d1x = 0, d1y = 0, d2x = 0, d2y = 0, K;
	float B[5];
	AllBernstin(n-1, u, B);
	for (i = 0; i <= n - 1; i++)
	{
		d1x += B[i] * (Px[i + 1] - Px[i]);
		d1y += B[i] * (Py[i + 1] - Py[i]);
	}
	d1x *= n;
	d1y *= n;

	AllBernstin(n - 2, u, B);
	for (i = 0; i <= n - 2; i++)
	{
		d2x += B[i] * (Px[i + 2] - 2 * Px[i + 1] + Px[i]);
		d2y += B[i] * (Py[i + 2] - 2 * Py[i + 1] + Py[i]);
	}
	d2x *= n*(n - 1);
	d2y *= n*(n - 1);

	K = fabs(d1x*d2y - d2x*d1y) / pow((d1x*d1x + d1y*d1y), 1.5);
	return K;
}


float DistanceOfBezierCeuve(int n, float* Px, float* Py)
{
	float u=0,d=0.0;
	float C[2];
	float x = Px[0], y = Py[0];
	 
	while (u<=1)
	{
		PointOnBezierCurve(n, u, Px, Py, C);
		d += sqrt((C[0]-x)*(C[0] - x)+ (C[1] - y)*(C[1] - y));
		x = C[0];
		y = C[1];
		u += 0.0001;
	}
	return d;
}


