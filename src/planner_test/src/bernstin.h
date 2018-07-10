#pragma once

void AllBernstin(int n, float u, float* B);

void PointOnBezierCurve(int n, float u, float* Px, float* Py,float* C);


float CurvatureOnBezierCeuve(int n,float u,float* Px,float* Py);

float DistanceOfBezierCeuve(int n, float* Px, float* Py);
