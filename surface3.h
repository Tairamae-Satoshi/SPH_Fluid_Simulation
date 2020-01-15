#pragma once
#include"vec.h"

class Surface3 {
public:
	//! Local-to-world transform.
	//Transform3 transform;
	Surface3();
	//! Default destructor.
	virtual ~Surface3();

	Point3D p1;
	Point3D p2;

	//! Returns the closest point from the given point \p otherPoint to the
	//! surface.
	Vector3D closestPoint(const Vector3D& otherPoint) const;

	//! Returns the closest distance from the given point \p otherPoint to the
	//! point on the surface.
	double closestDistance(const Vector3D& otherPoint) const;


	//! Returns the normal to the closest point on the surface from the given
	//! point \p otherPoint.
	Vector3D closestNormal(const Vector3D& otherPoint) const;

	//! Updates internal spatial query engine.
	virtual void updateQueryEngine();

	//! Returns true if bounding box can be defined.
	virtual bool isBounded() const;

	//! Returns true if the surface is a valid geometry.
	virtual bool isValidGeometry() const;

	//! Returns true if \p otherPoint is inside the volume defined by the
	//! surface.
	bool isInside(const Vector3D& otherPoint) const;

protected:
};

//! Shared pointer for the Surface3 type.
typedef std::shared_ptr<Surface3> Surface3Ptr;

Surface3::~Surface3() {}

Vector3D Surface3::closestPoint(const Vector3D& otherPoint) const {
	//return transform.toWorld(closestPointLocal(transform.toLocal(otherPoint)));
	return Vector3D(0, 0, 0);
}

double Surface3::closestDistance(const Vector3D& otherPoint) const {
	//return closestDistanceLocal(transform.toLocal(otherPoint));
	return 0;
}

Vector3D Surface3::closestNormal(const Vector3D& otherPoint) const {
//	auto result = transform.toWorldDirection(
	//	closestNormalLocal(transform.toLocal(otherPoint)));
	//result *= (isNormalFlipped) ? -1.0 : 1.0;
	//return result;
	return Vector3D(0, 0, 0);

}

void Surface3::updateQueryEngine() {
	// Do nothing
}

bool Surface3::isBounded() const { return true; }

bool Surface3::isValidGeometry() const { return true; }

bool Surface3::isInside(const Vector3D& otherPoint) const {
	//return isNormalFlipped == !isInsideLocal(transform.toLocal(otherPoint));
	return true;
}


