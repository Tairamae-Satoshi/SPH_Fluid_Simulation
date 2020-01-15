#pragma once
#pragma once

#include"vec.h"

#include<functional>

class Collider3 {
public:
	typedef std::function<void(Collider3*, double, double)> OnBeginUpdateCallback;

	Collider3();

	virtual ~Collider3();

	void resolveCollision(
		double radius,
		double restitutionCoefficient,
		Vector3D* position,
		Vector3D* velocity
	);

	void update(double currentTimeInSeconds, double timeIntervalInSeconds);
	void setOnBeginUpdateCallback(const OnBeginUpdateCallback& callback);

protected:
	struct ColliderQueryResult final {
		double distance;
		Vector3D point;
		Vector3D normal;
		Vector3D velocity;
	};

	bool isPenetrating(
		const ColliderQueryResult& colliderPoint,
		const Vector3D& position,
		double radius);

private:
	double _frictionCoeffient = 0.0;
	OnBeginUpdateCallback _onUpdateCallback;
};

typedef std::shared_ptr<Collider3> Collider3Ptr;

Collider3::Collider3() {}

Collider3::~Collider3() {}

void Collider3::resolveCollision(
	double radius,
	double restitutionCoefficient,
	Vector3D* newPosition,
	Vector3D* newVelocity) {
	restitutionCoefficient = 0.85;
	if ((*newPosition)[1]-radius < 0 && (*newVelocity)[1] < 0)
	{
		(*newVelocity)[1] *= -restitutionCoefficient;
	}
	if ((*newPosition)[1] + radius > 1 && (*newVelocity)[1] > 0)
	{
		(*newVelocity)[1] *= -restitutionCoefficient;
	}
	if ((*newPosition)[0] - radius < 0 && (*newVelocity)[0] < 0){
		(*newVelocity)[0] *= -restitutionCoefficient;
	}
	if ((*newPosition)[0] + radius > 1 && (*newVelocity)[0] > 0)
	{
		(*newVelocity)[0] *= -restitutionCoefficient;
	}
	if ((*newPosition)[2] - radius < 0 && (*newVelocity)[2] < 0) {
		(*newVelocity)[2] *= -restitutionCoefficient;

	}
	if ((*newPosition)[2]+radius > 1 && (*newVelocity)[2] > 0){
		(*newVelocity)[2] *= -restitutionCoefficient;
	}
}

bool Collider3::isPenetrating(const ColliderQueryResult& colliderPoint,
	const Vector3D& position, double radius) {
	return false;
}

void Collider3::update(double currentTimeInSeconds, double timeIntervalInSeconds) {
}

void Collider3::setOnBeginUpdateCallback(const OnBeginUpdateCallback& callback) {
	_onUpdateCallback = callback;
}

