#pragma once

#include"particle_system_solver3.h"
#include"sph_system_data3.h"

#include<tbb/tbb.h>

static double kTimeStepLimitBySpeedFactor = 0.4;
static double kTimeStepLimitByForceFactor = 0.25;

double computePressureFromEos(double density, double targetDensity, double eosScale, double eosExponent, double negativePressureScale) {
	double p = eosScale / eosExponent * (std::pow((density / targetDensity), eosExponent) - 1.0);

	if (p < 0)
	{
		p *= negativePressureScale;
	}

	return p;
}

class SphSystemSolver3 : public ParticleSystemSolver3
{
public:
	SphSystemSolver3();

	virtual ~SphSystemSolver3();

	SphSystemData3Ptr sphData() const;
protected:
	void accumulateForces(double timeStepInSeconds) override;

	void onBeginAdvanceTimeStep(double timeStepInSeconds) override;

	void onEndAdvanceTimeStep(double timeStepInSeconds) override;

	unsigned int numberOfSubTimeSteps(
		double timeIntervalInSeconds) const override;

	virtual void accumulateNonPressureForces(double timeStepInSeconds);

	virtual void accumulatePressureForce(double timeStepInSeconds);

	void accumulatePressureForce(
		const std::vector<Vector3D>& positions,
		const std::vector<double>& densities,
		const std::vector<double>& pressures,
		std::vector<Vector3D>& pressureForces);

	void computePressure();

	void accumulateViscosityForce();

	void computePseudoViscosity(double timeStepInSeconds);

	double negativePressureScale() const;

	void setTimeStepLimitScale(double newScale);

private:
	double _eosExponent = 7.0;
	double _negativePressureScale = 0.0;
	double _viscosityCoefficient = 0.01;
	double _pseudoViscosityCoefficient = 10.0;
	double _speedOfSound = 100.0;
	double _timeStepLimitScale = 1.0;
};

SphSystemSolver3::SphSystemSolver3()
{
	setParticleSystemData(std::make_shared<SphSystemData3>());
}

SphSystemSolver3::~SphSystemSolver3()
{
}

SphSystemData3Ptr SphSystemSolver3::sphData() const {
	return std::dynamic_pointer_cast<SphSystemData3>(particleSystemData());
}

void SphSystemSolver3::accumulateForces(double timeStepInSeconds) {
	accumulateNonPressureForces(timeStepInSeconds);
	accumulatePressureForce(timeStepInSeconds);
}

void SphSystemSolver3::accumulateNonPressureForces(double timeStepInSeconds) {
	ParticleSystemSolver3::accumulateForces(timeStepInSeconds);
	accumulateViscosityForce();
}

void SphSystemSolver3::accumulatePressureForce(double timeStepInSeconds) {
	auto particles = sphData();
	auto x = particles->positions();
	auto d = particles->densities();
	auto p = particles->pressures();
	auto& f = particles->forces();

	computePressure();
	accumulatePressureForce(x, d, p, f);

}

void SphSystemSolver3::accumulatePressureForce(
	const std::vector<Vector3D>& positions,
	const std::vector<double>& densities,
	const std::vector<double>& pressures,
	std::vector<Vector3D>& pressureForces) {
	auto particles = sphData();
	size_t numberOfParticles = particles->numberOfParticles();

	const double mass = particles->mass();
	const double massSquared = mass * mass;
	const SphSpikyKernel3 kernel(particles->kernelRadius());

	tbb::parallel_for(tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i) {
				const auto& neighbors = particles->neighborLists()[i];

				for (size_t j : neighbors) {
					double distance = dist(positions[i], positions[j]);
					if (distance > 0.0) {
						Vector3D dir = (positions[j] - positions[i]) / distance;
						pressureForces[i] -= massSquared *
							(pressures[i] / (densities[i] * densities[i])
								+ pressures[j] / (densities[j] * densities[j]))
							* kernel.gradient(distance, dir);
					}
				}
			}
		});
}


void SphSystemSolver3::onBeginAdvanceTimeStep(double timeStepInSeconds) {
	auto particles = sphData();

	particles->buildNeighborSearcher(particles->kernelRadius());
	particles->buildNeighborLists(particles->kernelRadius());
	particles->updateDensities();
}

void SphSystemSolver3::onEndAdvanceTimeStep(double timeStepInSeconds) {
	computePseudoViscosity(timeStepInSeconds);
}

void SphSystemSolver3::computePressure() {
	auto particles = sphData();
	size_t numberOfParticles = particles->numberOfParticles();
	auto d = particles->densities();
	auto& p = particles->pressures();

	const double targetDensity = particles->targetDensity();

	const double eosScale = targetDensity * (_speedOfSound * _speedOfSound) / _eosExponent;

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i < b.end(); ++i) {
				p[i] = computePressureFromEos(d[i], targetDensity, eosScale, _eosExponent, _negativePressureScale);
			}
		}
	);
}

void SphSystemSolver3::accumulateViscosityForce() {
	auto particles = sphData();
	size_t numberOfParticles = particles->numberOfParticles();
	auto x = particles->positions();
	auto v = particles->velocities();
	auto d = particles->densities();
	auto& f = particles->forces();

	const double massSquared = particles->mass() * particles->mass();
	const SphSpikyKernel3 kernel(particles->kernelRadius());

	tbb::parallel_for(tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i < b.end(); ++i) {
				const auto& neighbors = particles->neighborLists()[i];
				for (size_t j : neighbors) {
					double distance = dist(x[i], x[j]);

					f[i] += _viscosityCoefficient * massSquared
						* (v[j] - v[i]) / d[j]
						* kernel.secondDerivative(distance);
				}
			}
		});
}

void SphSystemSolver3::computePseudoViscosity(double timeStepInSeconds) {
	//return;
	auto particles = sphData();
	size_t numberOfParticles = particles->numberOfParticles();
	auto& x = particles->positions();
	auto& v = particles->velocities();
	auto& d = particles->densities();

	const double mass = particles->mass();
	const SphSpikyKernel3 kernel(particles->kernelRadius());

	std::vector<Vector3D> smoothedVelocities(numberOfParticles);

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				double weightSum = 0.0;
				Vector3D smoothedVelocity(0.0,0.0,0.0);

				const auto& neighbors = particles->neighborLists()[i];
				for (size_t j : neighbors) {
					double distance = dist(x[i], x[j]);
					
					double wj = mass / d[j] * kernel(distance);
					weightSum += wj;
					smoothedVelocity += wj * v[j];
				}

				double wi = mass / d[i];

				weightSum += wi;
				smoothedVelocity += wi * v[i];

				if (weightSum > 0.0) {
					smoothedVelocity /= weightSum;
				}

				smoothedVelocities[i] = smoothedVelocity;
			}
		});

	double factor = timeStepInSeconds * _pseudoViscosityCoefficient;
	factor = clamp(factor, 0.0, 1.0);

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				v[i] = lerp(
					v[i], smoothedVelocities[i], factor);
			}
		});
}

unsigned int SphSystemSolver3::numberOfSubTimeSteps(
	double timeIntervalInSeconds) const {
	auto particles = sphData();
	size_t numberOfParticles = particles->numberOfParticles();
	auto f = particles->forces();

	const double kernelRadius = particles->kernelRadius();
	const double mass = particles->mass();

	double maxForceMagnitude = 0.0;

	for (size_t i = 0; i < numberOfParticles; ++i) {
		maxForceMagnitude = max(maxForceMagnitude, mag(f[i]));
	}

	double timeStepLimitBySpeed
		= kTimeStepLimitBySpeedFactor * kernelRadius / _speedOfSound;
	double timeStepLimitByForce
		= kTimeStepLimitByForceFactor
		* std::sqrt(kernelRadius * mass / maxForceMagnitude);

	double desiredTimeStep
		= _timeStepLimitScale
		* std::min(timeStepLimitBySpeed, timeStepLimitByForce);

	return static_cast<unsigned int>(
		std::ceil(timeIntervalInSeconds / desiredTimeStep));
}

double SphSystemSolver3::negativePressureScale() const {
	return _negativePressureScale;
}

void SphSystemSolver3::setTimeStepLimitScale(double newScale) {
	_timeStepLimitScale = std::max(newScale, 0.0);
}