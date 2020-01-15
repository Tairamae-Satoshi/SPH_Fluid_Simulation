#pragma once

#include"vec.h"
#include<functional>
class PointGenerator3 {
public:
	PointGenerator3();

	virtual ~PointGenerator3();

	void generate(
		const Vector3D& lowerCorner,
		const Vector3D& upperCorner,
		double spacing,
		std::vector<Vector3D>* points) const;

	virtual void forEachPoint(
		const Vector3D& lowerCorner,
		const Vector3D& upperCorner,
		double spacing,
		const std::function<bool(const Vector3D&)>& callback) const;
};

typedef std::shared_ptr<PointGenerator3> PointGenerator3Ptr;

PointGenerator3::PointGenerator3() {
}

PointGenerator3::~PointGenerator3() {
}

void PointGenerator3::generate(
	const Vector3D& lowerCorner,
	const Vector3D& upperCorner,
	double spacing,
	std::vector<Vector3D>* points) const {
	forEachPoint(
		lowerCorner,
		upperCorner,
		spacing,
		[&points](const Vector3D& point) {
			points->push_back(point);
			return true;
		});
}

void PointGenerator3::forEachPoint(
	const Vector3D& lowerCorner,
	const Vector3D& upperCorner,
	double spacing,
	const std::function<bool(const Vector3D&)>& callback) const {
	double halfSpacing = spacing / 2.0;
	double boxWidth = upperCorner[0]- lowerCorner[0];
	double boxHeight = upperCorner[1] - lowerCorner[1];
	double boxDepth = upperCorner[2] - lowerCorner[2];

	Vector3D position;
	bool hasOffset = false;
	bool shouldQuit = false;
	for (int k = 0; k * halfSpacing <= boxDepth && !shouldQuit; ++k) {
		position[2] = k * halfSpacing + lowerCorner[2];

		double offset = (hasOffset) ? halfSpacing : 0.0;

		for (int j = 0; j * spacing + offset <= boxHeight && !shouldQuit; ++j) {
			position[1] = j * spacing + offset + lowerCorner[1];

			for (int i = 0; i * spacing + offset <= boxWidth; ++i) {
				position[0] = i * spacing + offset + lowerCorner[0];
				if (!callback(position)) {
					shouldQuit = true;
					break;
				}
			}
		}

		hasOffset = !hasOffset;
	}
}