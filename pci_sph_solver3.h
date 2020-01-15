#pragma once

#include"sph_system_solver3.h"

const double kDefaultTimeStepLimitScale = 5.0;

class PciSphSolver3 : public SphSystemSolver3{
 public:
	class Builder;

	//! Constructs a solver with empty particle set.
	PciSphSolver3();

	virtual ~PciSphSolver3();

	//! Returns max allowed density error ratio.
	double maxDensityErrorRatio() const;

	//!
	//! \brief Sets max allowed density error ratio.
	//!
	//! This function sets the max allowed density error ratio during the PCISPH
	//! iteration. Default is 0.01 (1%). The input value should be positive.
	//!
	void setMaxDensityErrorRatio(double ratio);

	//! Returns max number of iterations.
	unsigned int maxNumberOfIterations() const;

	void setMaxNumberOfIterations(unsigned int n);

 protected:
	 //! Accumulates the pressure force to the forces array in the particle
	 //! system.
	 void accumulatePressureForce(double timeIntervalInSeconds) override;

  private:
	 double _maxDensityErrorRatio = 0.01;
	 unsigned int _maxNumberOfIterations = 5;

	 ParticleSystemData3::VectorData _tempPositions;
	 ParticleSystemData3::VectorData _tempVelocities;
	 ParticleSystemData3::VectorData _pressureForces;
	 ParticleSystemData3::ScalarData _densityErrors;

	 double computeDelta(double timeStepInSeconds);
	 double computeBeta(double timeStepInSeconds);
};

//! Shared pointer type for the PciSphSolver3.
typedef std::shared_ptr<PciSphSolver3> PciSphSolver3Ptr;

PciSphSolver3::PciSphSolver3() {
	setTimeStepLimitScale(kDefaultTimeStepLimitScale);
}

PciSphSolver3::~PciSphSolver3() {
}

double PciSphSolver3::maxDensityErrorRatio() const {
	return _maxDensityErrorRatio;
}

void PciSphSolver3::setMaxDensityErrorRatio(double ratio) {
	_maxDensityErrorRatio = std::max(ratio, 0.0);
}

unsigned int PciSphSolver3::maxNumberOfIterations() const {
	return _maxNumberOfIterations;
}

void PciSphSolver3::setMaxNumberOfIterations(unsigned int n) {
	_maxNumberOfIterations = n;
}

void PciSphSolver3::accumulatePressureForce(
	double timeIntervalInseconds) {
	auto particles = sphData();
	const size_t numberOfParticles = particles->numberOfParticles();
	const double delta = computeDelta(timeIntervalInseconds);
	const double targetDensity = particles->targetDensity();
	const double mass = particles->mass();

	auto p = particles->pressures();
	auto d = particles->densities();
	auto x = particles->positions();
	auto v = particles->velocities();
	auto f = particles->forces();

	// Predicted density ds
	std::vector<double> ds(numberOfParticles);

	SphSpikyKernel3 kernel(particles->kernelRadius());

	// Initialize buffers
	tbb::parallel_for(tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i) {
				p[i] = 0.0;
				_pressureForces[i] = Vector3D(0.0, 0.0, 0.0);
				_densityErrors[i] = 0.0;
				ds[i] = d[i];
			}
		});

	unsigned int maxNumIter = 0;
	double maxDensityError;
	double densityErrorRatio = 0.0;
	for (size_t k = 0; k < _maxNumberOfIterations; k++)
	{
		// Predict velocity and position
		tbb::parallel_for(tbb::blocked_range<size_t>(0, numberOfParticles),
			[&](const tbb::blocked_range<size_t>& b) {
				for (size_t i = b.begin(); i != b.end(); ++i) {
					_tempVelocities[i] = v[i] + timeIntervalInseconds / mass * (f[i] + _pressureForces[i]);
					_tempPositions[i] = x[i] + timeIntervalInseconds * _tempVelocities[i];
				}
			});

		// Resolve collisions
		resolveCollision(
			_tempPositions,
			_tempVelocities);

		// Compute pressure from density error
		tbb::parallel_for(tbb::blocked_range<size_t>(0, numberOfParticles),
			[&](const tbb::blocked_range<size_t>& b) {
				for (size_t i = b.begin(); i != b.end(); ++i) {
					double weightSum = 0.0;
					const auto& neighbors = particles->neighborLists()[i];

					for (size_t j : neighbors) {
						double distance
							= dist(_tempPositions[j], _tempPositions[i]);
						weightSum += kernel(distance);
					}
					weightSum += kernel(0);

					double density = mass * weightSum;
					double densityError = (density - targetDensity);
					double pressure = delta * densityError;

					if (pressure < 0.0) {
						pressure *= negativePressureScale();
						densityError *= negativePressureScale();
					}

					p[i] += pressure;
					ds[i] = density;
					_densityErrors[i] = densityError;
				}
			});

		// Compute pressure gradient force
		SphSystemSolver3::accumulatePressureForce(x, ds, p, _pressureForces);

		// Compute max density error
		maxDensityError = 0.0;
		for (size_t i = 0; i < numberOfParticles; ++i)
		{
			maxDensityError = std::max(std::fabs(maxDensityError), std::fabs(_densityErrors[i]));
		}

		densityErrorRatio = maxDensityError / targetDensity;
		maxNumIter = k + 1;

		if (std::fabs(densityErrorRatio) < _maxDensityErrorRatio) {
			break;
		}
	}

	// Accumulate pressure force
	tbb::parallel_for(tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i) {
				f[i] += _pressureForces[i];
			}
		});
}

double PciSphSolver3::computeDelta(double timeStepInSeconds) {
	auto particles = sphData();
	const double kernelRadius = particles->kernelRadius();

	std::vector<Vector3D> points;
	PointGenerator3 pointsGenerator;
	Vector3D lowerCorner(-1.5*kernelRadius, -1.5 * kernelRadius, -1.5 * kernelRadius);
	Vector3D upperCorner(1.5 * kernelRadius, 1.5 * kernelRadius, 1.5 * kernelRadius);

	pointsGenerator.generate(lowerCorner, upperCorner, particles->targetSpacing(), &points);

	SphSpikyKernel3 kernel(kernelRadius);

	double denom = 0;
	Vector3D denom1;
	double denom2 = 0;

	for (size_t i = 0; i < points.size(); ++i) {
		const Vector3D& point = points[i];
		double distanceSquared = mag2(point);

		if (distanceSquared < kernelRadius * kernelRadius) {
			double distance = std::sqrt(distanceSquared);
			Vector3D direction =
				(distance > 0.0) ? point / distance : Vector3D();

			// grad(Wij)
			Vector3D gradWij = kernel.gradient(distance, direction);
			denom1 += gradWij;
			denom2 += dot(gradWij, gradWij);
		}
	}

	denom += -dot(denom1, denom1) - denom2;

	return (std::fabs(denom) > 0.0) ?
		-1 / (computeBeta(timeStepInSeconds) * denom) : 0;
}

double PciSphSolver3::computeBeta(double timeStepInSeconds) {
	auto particles = sphData();
	return 2.0 * pow(particles->mass() * timeStepInSeconds
		/ particles->targetDensity(),2);
}