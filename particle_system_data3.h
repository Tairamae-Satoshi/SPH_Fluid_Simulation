#pragma once


#include<vector>

#include<tbb/tbb.h>
#include "vec.h"
#include"point_hash_grid_searcher.h"

static const size_t kDefaultHashGridResolution = 16;

class ParticleSystemData3 {
public:
	typedef std::vector<double> ScalarData;
	typedef std::vector<Vector3D> VectorData;

	ParticleSystemData3();

	virtual ~ParticleSystemData3();
	explicit ParticleSystemData3(size_t numberOfParticles);
	void resize(size_t newNumberOfParticles);
	size_t numberOfParticles() const;
	size_t addScalarData(double initialVal = 0.0);
	size_t addVectorData(const Vector3D& initialVal = Vector3D(0, 0, 0));

	const VectorData& positions() const;
	VectorData& positions();
	const VectorData& velocities() const;
	VectorData& velocities();
	const VectorData& forces() const;
	VectorData& forces();

	void addParticle(
		const Vector3D& newPosition,
		const Vector3D& newVelocity,
		const Vector3D& newForce = Vector3D(0, 0, 0));
	void addParticles(
		const std::vector<Vector3D>& newPositions,
		const std::vector<Vector3D>& newVelocities,
		const std::vector<Vector3D>& newForces);

	const std::vector<double>& scalarDataAt(size_t idx) const;
	std::vector<double>& scalarDataAt(size_t idx);
	const std::vector<Vector3D>& vectorDataAt(size_t idx) const;
	std::vector<Vector3D>& vectorDataAt(size_t idx);

	
	double mass() const;
	virtual void setMass(double newMass);
	double radius() const;
	virtual void setRadius(double newRadius);

	void buildNeighborSearcher(double maxSearchRadius);
	void buildNeighborLists(double maxSearchRadius);

	const std::vector <std::vector<size_t>>& neighborLists() const;
	const PointHashGridSearcher3Ptr& neighborSearcher() const;

private:
	double _radius = 1e-3;
	double _mass = 1e-3;
	size_t _positionIdx;
	size_t _velocityIdx;
	size_t _forceIdx;
	size_t _numberOfParticles = 0;
	std::vector<ScalarData> _scalarDataList;
	std::vector<VectorData> _vectorDataList;
	PointHashGridSearcher3Ptr _neighborSearcher;
	std::vector <std::vector<size_t>> _neighborLists;
};

typedef std::shared_ptr<ParticleSystemData3> ParticleSystemData3Ptr;

ParticleSystemData3::ParticleSystemData3() 
	:ParticleSystemData3(0) {
}

ParticleSystemData3::ParticleSystemData3(size_t numberOfParticles) {
	_positionIdx = addVectorData();
	_velocityIdx = addVectorData();
	_forceIdx = addVectorData();

	_neighborSearcher = std::make_shared<PointHashGridSearcher3>(
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		2.0 * _radius);
	resize(numberOfParticles);
}

ParticleSystemData3::~ParticleSystemData3() {}

void ParticleSystemData3::resize(size_t newNumberOfParticles) {
	_numberOfParticles = newNumberOfParticles;

	for (auto& attr : _scalarDataList){
		attr.resize(newNumberOfParticles, 0.0);
	}

	for (auto& attr : _vectorDataList){
		attr.resize(newNumberOfParticles, Vector3D(0.0, 0.0, 0.0));
	}
}

size_t ParticleSystemData3::numberOfParticles() const {
	return _numberOfParticles;
}


size_t ParticleSystemData3::addScalarData(double initialVal) {
	size_t attrIdx = _scalarDataList.size();
	_scalarDataList.emplace_back(numberOfParticles(), initialVal);
	return attrIdx;
}

size_t ParticleSystemData3::addVectorData(const Vector3D& initialVal) {
	size_t attrIdx = _vectorDataList.size();
	_vectorDataList.emplace_back(numberOfParticles(), initialVal);
	return attrIdx;
}

double ParticleSystemData3::mass() const {
	return _mass;
}

void ParticleSystemData3::setMass(double newMass) {
	_mass = std::max(newMass, 0.0);
}

double ParticleSystemData3::radius() const {
	return _radius;
}

void ParticleSystemData3::setRadius(double newRadius) {
	_radius = std::max(newRadius, 0.0);
}

const std::vector<Vector3D>& ParticleSystemData3::positions() const {
	return vectorDataAt(_positionIdx);
}

std::vector<Vector3D>& ParticleSystemData3::positions() {
	return vectorDataAt(_positionIdx);
}

const std::vector<Vector3D>& ParticleSystemData3::velocities() const {
	return vectorDataAt(_velocityIdx);
}

std::vector<Vector3D>& ParticleSystemData3::velocities() {
	return vectorDataAt(_velocityIdx);
}

const std::vector<Vector3D>& ParticleSystemData3::forces() const {
	return vectorDataAt(_forceIdx);
}

std::vector<Vector3D>& ParticleSystemData3::forces() {
	return vectorDataAt(_forceIdx);
}

const std::vector<double>& ParticleSystemData3::scalarDataAt(
	size_t idx) const {
	return _scalarDataList[idx];
}

std::vector<double>& ParticleSystemData3::scalarDataAt(size_t idx) {
	return _scalarDataList[idx];
}

const std::vector<Vector3D>& ParticleSystemData3::vectorDataAt(
	size_t idx) const {
	return _vectorDataList[idx];
}

std::vector<Vector3D>& ParticleSystemData3::vectorDataAt(size_t idx) {
	return _vectorDataList[idx];
}

void ParticleSystemData3::addParticle(
	const Vector3D& newPosition,
	const Vector3D& newVelocity,
	const Vector3D& newForce) {
	std::vector<Vector3D> newPositions = { newPosition };
	std::vector<Vector3D> newVelocities = { newVelocity };
	std::vector<Vector3D> newForces = { newForce };

	addParticles(newPositions, newVelocities, newForces);
}

void ParticleSystemData3::addParticles(
	const std::vector<Vector3D>& newPositions,
	const std::vector<Vector3D>& newVelocities,
	const std::vector<Vector3D>& newForces) {
	
	size_t oldNumberOfParticles = numberOfParticles();
	size_t newNumberOfParticles = oldNumberOfParticles + newPositions.size();

	resize(newNumberOfParticles);

	auto& pos = positions();
	auto& vel = velocities();
	auto& frc = forces();

	size_t n = newPositions.size();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, n),
		[&](const tbb::blocked_range<size_t>& b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				pos[i + oldNumberOfParticles] = newPositions[i];
			}
		});

	if (newVelocities.size() > 0) {
		tbb::parallel_for(
			tbb::blocked_range<size_t>(0, n),
			[&](const tbb::blocked_range<size_t>& b) {
				for (size_t i = b.begin(); i != b.end(); ++i)
				{
					vel[i + oldNumberOfParticles] = newVelocities[i];
				}
			}
		);
	}

	if (newForces.size() > 0) {
		tbb::parallel_for(
			tbb::blocked_range<size_t>(0, n),
			[&](const tbb::blocked_range<size_t>& b) {
				for (size_t i = b.begin(); i != b.end(); ++i)
				{
					frc[i + oldNumberOfParticles] = newForces[i];
				}
			}
		);
	}
}

void ParticleSystemData3::buildNeighborSearcher(double maxSearchRadius) {
	_neighborSearcher = std::make_shared<PointHashGridSearcher3>(
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		2.0 * maxSearchRadius);
	_neighborSearcher->build(positions());
}

void ParticleSystemData3::buildNeighborLists(double maxSearchRadius) {
	_neighborLists.resize(numberOfParticles());
	
	auto points = positions();
	for (size_t i = 0; i < numberOfParticles(); ++i)
	{
		Vector3D origin = points[i];
		_neighborLists[i].clear();

		_neighborSearcher->forEachNearbyPoint(
			origin,
			maxSearchRadius,
			[&](size_t j, const Vector3D&) {
				if (i != j){
					_neighborLists[i].push_back(j);
				}
			}
		);
	}
}

const std::vector <std::vector<size_t>>& ParticleSystemData3::neighborLists() const {
	return _neighborLists;
}

const PointHashGridSearcher3Ptr& ParticleSystemData3::neighborSearcher() const {
	return _neighborSearcher;
}

