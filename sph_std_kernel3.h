#pragma once
#define _USE_MATH_DEFINES

#include"vec.h"

#include<math.h>
struct SphStdKernel3 {
	double h, h2, h3, h5;

	SphStdKernel3();

	explicit SphStdKernel3(double kernelRadius);

	double operator()(double distance) const;

	double firstDerivative(double distance) const;

	Vector3D gradient(double distance, const Vector3D& direction) const;

	double secondDerivative(double distance) const;

};

inline SphStdKernel3::SphStdKernel3()
	: h(0), h2(0), h3(0), h5(0) {}

inline SphStdKernel3::SphStdKernel3(double kernelRadius)
	: h(kernelRadius), h2(h* h), h3(h2* h), h5(h2* h3) {}

inline double SphStdKernel3::operator()(double distance) const {
	if (distance * distance >= h2){
		return 0.0;
	}
	else{
		double x = 1.0 - distance * distance / h2;
		return 315.0 / (64.0 * M_PI * h3) * x * x * x;
	}
}

inline double SphStdKernel3::firstDerivative(double distance) const {
	if (distance >= h) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance * distance / h2;
		return -945.0 / (32.0 * M_PI * h5) * distance * x * x;
	}
}


inline Vector3D SphStdKernel3::gradient(double distance, const Vector3D& directionTocenter) const {
	return -firstDerivative(distance) * directionTocenter;
}

inline double SphStdKernel3::secondDerivative(double distance) const {
	if (distance * distance >= h2){
		return 0.0;
	}
	else{
		double x = distance * distance / h2;
		return 945.0 / (32.0 * M_PI * h5) * (1 - x) * (5 * x - 1);
	}
}