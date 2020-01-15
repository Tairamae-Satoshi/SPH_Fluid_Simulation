#pragma once

#include"vec.h"

struct SphSpikyKernel3
{
	double h, h2, h3, h4, h5;

	SphSpikyKernel3();

	explicit SphSpikyKernel3(double kernelRadius);

	double operator()(double distance) const;

	double firstDerivative(double distance) const;

	Vector3D gradient(double distance, const Vector3D& direction) const;

	double secondDerivative(double distance) const;

	double laplacian(double distance) const;
};

inline SphSpikyKernel3::SphSpikyKernel3()
	: h(0), h2(0), h3(0), h4(0), h5(0) {}

inline SphSpikyKernel3::SphSpikyKernel3(double h_)
	: h(h_), h2(h* h), h3(h2* h), h4(h2* h2), h5(h3* h2) {}

inline double SphSpikyKernel3::operator()(double distance) const {
	if (distance >= h) {
		return 0.0;

	}
	else {
		double x = 1.0 - distance / h;
		return 15.0 / (M_PI * h3) * x * x * x;
	}
}

inline double SphSpikyKernel3::firstDerivative(double distance) const {
	if (distance >= h) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance / h;
		return -45.0 / (M_PI * h4) * x * x;

	}

}

inline Vector3D SphSpikyKernel3::gradient(
	double distance,
	const Vector3D& directionToCenter) const {
	return -firstDerivative(distance) * directionToCenter;

}

inline double SphSpikyKernel3::secondDerivative(double distance) const {
	if (distance >= h) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance / h;
		return 90.0 / (M_PI * h5) * x;
	}

}

double SphSpikyKernel3::laplacian(double distance) const {
	return secondDerivative(distance);
}
