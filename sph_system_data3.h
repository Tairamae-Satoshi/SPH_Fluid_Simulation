#pragma once

#include"particle_system_data3.h"
#include"sph_std_kernel3.h"
#include"sph_spiky_kernel3.h"
#include"point_generator3.h"

#include<vector>

constexpr double kWaterDensity = 1000.0;

class SphSystemData3 : public ParticleSystemData3{
public:
	SphSystemData3();
	explicit SphSystemData3(size_t numberOfParticles);
	virtual ~SphSystemData3();

	const ScalarData& densities() const;
	ScalarData& densities();
	const ScalarData& pressures() const;
	ScalarData& pressures();
	double targetDensity() const;
	void setTargetSpacing(double spacing);
	//Vector3D interpolate(const Vector3D& origin, const std::vector<Vector3D>& values) const;

	void updateDensities();

	double sumOfKernelNearby(const Vector3D& position) const;

	/*Vector3D gradientAt(size_t i, const std::vector<double>& values) const;

	double laplacianAt(size_t i, const std::vector<double>& values) const;*/

	double kernelRadius() const;
	double targetSpacing() const;
	void setRadius(double newRadius) override;
	void setMass(double newMass) override;
	void computeMass();

private:
	double _targetDensity = kWaterDensity;
	double _targetSpacing = 0.05;
	double _kernelRadiusOverTargetSpacing = 1.8;
	size_t _pressureIdx;
	size_t _densityIdx;
	double _kernelRadius;

};

typedef std::shared_ptr<SphSystemData3> SphSystemData3Ptr;

SphSystemData3::SphSystemData3():SphSystemData3(0)
{
}

SphSystemData3::SphSystemData3(size_t numberOfParticles) 
	: ParticleSystemData3(numberOfParticles) {
	_densityIdx = addScalarData();
	_pressureIdx = addScalarData();

	setTargetSpacing(_targetSpacing);
}

SphSystemData3::~SphSystemData3()
{
}

void SphSystemData3::setRadius(double newRadius) {
	// Interpret it as setting target spacing
	setTargetSpacing(newRadius);
}

void SphSystemData3::setMass(double newMass) {
	double incRatio = newMass / mass();
	_targetDensity *= incRatio;
	ParticleSystemData3::setMass(newMass);
}

void SphSystemData3::computeMass() {
	std::vector<Vector3D> points;
	PointGenerator3 pointsGenerator;

	pointsGenerator.generate(
		Vector3D(-1.5 * _kernelRadius, -1.5 * _kernelRadius, -1.5 * _kernelRadius),
		Vector3D(1.5 * _kernelRadius, 1.5 * _kernelRadius, 1.5 * _kernelRadius),
		_targetSpacing, 
		&points);

	double maxNumberDensity = 0.0;
	SphStdKernel3 kernel(_kernelRadius);

	for (size_t i = 0; i < points.size(); ++i) {
		const Vector3D& point = points[i];
		double sum = 0.0;

		for (size_t j = 0; j < points.size(); ++j) {
			const Vector3D& neighborPoint = points[j];
			sum += kernel(dist(point, neighborPoint));
		}

		maxNumberDensity = std::max(maxNumberDensity, sum);
	}

	double newMass = _targetDensity / maxNumberDensity;

	ParticleSystemData3::setMass(newMass);

	std::cout << mass() << std::endl;
}


const std::vector<double>& SphSystemData3::densities() const {
	return scalarDataAt(_densityIdx);
}

std::vector<double>& SphSystemData3::densities() {
	return scalarDataAt(_densityIdx);
}

const std::vector<double>& SphSystemData3::pressures() const {
	return scalarDataAt(_pressureIdx);
}

std::vector<double>& SphSystemData3::pressures() {
	return scalarDataAt(_pressureIdx);
}

double SphSystemData3::targetDensity() const { 
	return _targetDensity; 
}

void SphSystemData3::setTargetSpacing(double spacing) {
	ParticleSystemData3::setRadius(spacing);

	_targetSpacing = spacing;
	_kernelRadius = _kernelRadiusOverTargetSpacing * _targetSpacing;

	computeMass();
}

double SphSystemData3::kernelRadius() const
{
	return _kernelRadius;
}

double SphSystemData3::targetSpacing() const {
	return _targetSpacing;
}

void SphSystemData3::updateDensities() {
	auto p = positions();
	auto& d = densities();
	const double m = mass();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles()),
		[&](const tbb::blocked_range<size_t> b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				double sum = sumOfKernelNearby(p[i]);
				d[i] = m * sum;
			}
		}
	);
}

double SphSystemData3::sumOfKernelNearby(const Vector3D& origin) const {
	double sum = 0.0;
	SphStdKernel3 kernel(_kernelRadius);

	neighborSearcher()->forEachNearbyPoint(
		origin,
		_kernelRadius,
		[&](size_t, const Vector3D& neighborPosition) {
			double distance = dist(origin, neighborPosition);
			sum += kernel(distance);
		}
	);
	return sum;
}
