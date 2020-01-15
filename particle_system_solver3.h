#pragma once

#include"particle_system_data3.h"
#include"physics_animation.h"
#include"collider3.h"

#include<vector>
#include<tbb/tbb.h>

class ParticleSystemSolver3 : public PhysicsAnimation {
private:
	ParticleSystemData3Ptr _particleSystemData;

	ParticleSystemData3::VectorData _newPositions;

	ParticleSystemData3::VectorData _newVelocities;

	void beginAdvanceTimeStep(double timeStepInSeconds);

	void endAdvanceTimeStep(double timeStepInSeconds);

	void timeIntegration(double timeIntervalInSeconds);

	void updateCollider(double timeStepInSeconds);


protected:
	void resolveCollision();

	void resolveCollision(std::vector<Vector3D>& newPositions, std::vector<Vector3D>& newVelocities);

	void onInitialize() override;

	void onAdvanceTimeStep(double timeIntervalInSeconds) override;

	virtual void accumulateForces(double timeStepInSeconds);

	void accumulateExternalForces();

	virtual void onBeginAdvanceTimeStep(double timeStepInSeconds);

	virtual void onEndAdvanceTimeStep(double timeStepInSeconds);

	void setParticleSystemData(const ParticleSystemData3Ptr& newParticles);

	Collider3Ptr _collider;
	double _dragCoefficient = 1e-4;
	double _restitutionCoefficient = 0.0;
	Vector3D _gravity = Vector3D(0.0, -9.8, 0.0);

public:
	ParticleSystemSolver3();
	ParticleSystemSolver3(double radius, double mass);
	virtual ~ParticleSystemSolver3();
	double dragCoefficient() const;
	void setDragCoefficient(double newDragCoefficient);
	double restitutionCoefficient() const;
	void setRestitutionCoefficient(double newRestitutionCoefficient);
	const Vector3D& gravity() const;
	void setGravity(const Vector3D& newGravity);
	const ParticleSystemData3Ptr& particleSystemData() const;
}; 

ParticleSystemSolver3::ParticleSystemSolver3()
	: ParticleSystemSolver3(1e-3, 1e-3) {
}


ParticleSystemSolver3::ParticleSystemSolver3(
	double radius,
	double mass) {
	_particleSystemData = std::make_shared<ParticleSystemData3>();
	_collider = std::make_shared<Collider3>();
	_particleSystemData->setRadius(radius);
	_particleSystemData->setMass(mass);
}


ParticleSystemSolver3::~ParticleSystemSolver3() {
}

double ParticleSystemSolver3::dragCoefficient() const {
	return _dragCoefficient;
}

void ParticleSystemSolver3::setDragCoefficient(double newDragCoefficient) {
	_dragCoefficient = max(newDragCoefficient, 0.0);
}

double ParticleSystemSolver3::restitutionCoefficient() const {
	return _restitutionCoefficient;
}

void ParticleSystemSolver3::setRestitutionCoefficient(
	double newRestitutionCoefficient) {
	_restitutionCoefficient = clamp(newRestitutionCoefficient, 0.0, 1.0);
}

const Vector3D& ParticleSystemSolver3::gravity() const {
	return _gravity;
}

void ParticleSystemSolver3::setGravity(const Vector3D& newGravity) {
	_gravity = newGravity;
}

const ParticleSystemData3Ptr&
ParticleSystemSolver3::particleSystemData() const {
	return _particleSystemData;
}

void ParticleSystemSolver3::onInitialize() {
	
	auto particles = particleSystemData();
	const double radius = particles->radius();
	const double diameter = 2 * particles->radius();
	std::vector<Vector3D> positions;
	std::vector<Vector3D> velocities;
	std::vector<Vector3D> forces;
	const double length = 1;
	const double width = 1;
	const double height = 0.5;
	for (double i = radius; i < length; i+= diameter*0.8)
	{
		for (double j = radius; j < width; j += diameter * 0.8)
		{
			for (double k = radius; k < height; k += diameter * 0.8)
			{

				positions.push_back(Vector3D(i, j, k));
				velocities.push_back(Vector3D(0, 0, 0));
				forces.push_back(Vector3D(0, 0, 0));
			}
		}
	}

	particles->addParticles(positions, velocities, forces);
}

void ParticleSystemSolver3::onAdvanceTimeStep(double timeIntervalInSeconds) {	
	beginAdvanceTimeStep(timeIntervalInSeconds);

	accumulateForces(timeIntervalInSeconds);
	timeIntegration(timeIntervalInSeconds);
	resolveCollision();
	
	endAdvanceTimeStep(timeIntervalInSeconds);

}

void ParticleSystemSolver3::accumulateForces(double timeStepInSeconds) {
	accumulateExternalForces();
}

void ParticleSystemSolver3::accumulateExternalForces() {
	size_t n = _particleSystemData->numberOfParticles();
	auto& forces = _particleSystemData->forces();
	const double mass = _particleSystemData->mass();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, n),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				// Gravity
				Vector3D force = mass * _gravity;

				forces[i] += force;
			}
		}
	);
}

void ParticleSystemSolver3::timeIntegration(double timeIntervalInSeconds) {
	size_t n = _particleSystemData->numberOfParticles();
	auto forces = _particleSystemData->forces();
	auto& velocities = _particleSystemData->velocities();
	auto& positions = _particleSystemData->positions();
	const double mass = _particleSystemData->mass();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, n),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i) {
				// Integrate velocity first
				Vector3D& newVelocity = _newVelocities[i];
				_newVelocities[i] = velocities[i] + timeIntervalInSeconds * forces[i] / mass;

				// Integrate position.
				Vector3D& newPosition = _newPositions[i];
				newPosition = positions[i] + timeIntervalInSeconds * newVelocity;
			}
		});
}

void ParticleSystemSolver3::beginAdvanceTimeStep(double timeStepInSeconds) {
	// Allocate buffers
	size_t n = _particleSystemData->numberOfParticles();
	_newPositions.resize(n);
	_newVelocities.resize(n);

	// Clear forces
	auto& forces = _particleSystemData->forces();
	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, forces.size()),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i) {
				forces[i] = Vector3D(0.0, 0.0, 0.0);
			}
		}
	);

	onBeginAdvanceTimeStep(timeStepInSeconds);
}
	
void ParticleSystemSolver3::endAdvanceTimeStep(double timeStepInSeconds) {
	// Update data
	size_t n = _particleSystemData->numberOfParticles();
	auto& positions = _particleSystemData->positions();
	auto& velocities = _particleSystemData->velocities();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, n),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				positions[i] = _newPositions[i];
				velocities[i] = _newVelocities[i];
			}
		}
	);
	
	onEndAdvanceTimeStep(timeStepInSeconds);
}

void ParticleSystemSolver3::resolveCollision() {
	resolveCollision(_newPositions, _newVelocities);
}

void ParticleSystemSolver3::resolveCollision(std::vector<Vector3D>& newPositions, std::vector<Vector3D>& newVelocities) {
	if (_collider != nullptr) {
		size_t numberOfParticles = _particleSystemData->numberOfParticles();
		const double radius = _particleSystemData->radius();

		tbb::parallel_for(
			tbb::blocked_range<size_t>(size_t(0), numberOfParticles),
			[&](const tbb::blocked_range<size_t>& b) {
				for (size_t i = b.begin(); i < b.end(); ++i) {
					_collider->resolveCollision(radius, _restitutionCoefficient, &newPositions[i], &newVelocities[i]);
				}
			});
	}
}


void ParticleSystemSolver3::updateCollider(double timeStepInSeconds) {
}

void ParticleSystemSolver3::onBeginAdvanceTimeStep(double timeStepInSeconds) {
}

void ParticleSystemSolver3::onEndAdvanceTimeStep(double timeStepInSeconds) {
}

void ParticleSystemSolver3::setParticleSystemData(
	const ParticleSystemData3Ptr& newParticles) {
	_particleSystemData = newParticles;
}