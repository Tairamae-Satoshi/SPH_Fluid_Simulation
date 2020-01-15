#pragma once

#include"point3.h"

class PointHashGridSearcher3
{
public:
	typedef std::function<void(size_t, const Vector3D&)> ForEachNearbyPointFunc;

	PointHashGridSearcher3(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacing);
	~PointHashGridSearcher3();

	void build(const std::vector<Vector3D>& points);

	void forEachNearbyPoint(const Vector3D& origin, double radius, const ForEachNearbyPointFunc& callback) const;

private:
	double _gridSpacing = 1.0;
	Point3I _resolution = Point3I(1, 1, 1);
	std::vector<Vector3D> _points;
	std::vector<std::vector<size_t>> _buckets;

	Point3I getBucketIndex(const Vector3D& position) const;

	size_t getHashKeyFromPosition(const Vector3D& position) const;

	size_t getHashKeyFromBucketIndex(const Point3I& bucketIndex) const;

	void getNearbyKeys(const Vector3D& position, size_t* nearbyKeys) const;
};

typedef std::shared_ptr<PointHashGridSearcher3> PointHashGridSearcher3Ptr;

PointHashGridSearcher3::PointHashGridSearcher3(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacing)
{
	_gridSpacing = gridSpacing;
	_resolution.x = std::max(resolutionX, size_t(1));
	_resolution.y = std::max(resolutionY, size_t(1));
	_resolution.z = std::max(resolutionZ, size_t(1));

}

PointHashGridSearcher3::~PointHashGridSearcher3()
{
}

void PointHashGridSearcher3::build(const std::vector<Vector3D>& points)
{
	_buckets.clear();
	_points.clear();

	// Allocate memory chuncks
	_buckets.resize(_resolution.x * _resolution.y * _resolution.z);
	_points.resize(points.size());

	if (points.size() == 0)
	{
		return;
	}

	// Allocate memory chuncks
	_buckets.resize(_resolution.x * _resolution.y * _resolution.z);
	_points.resize(points.size());

	// Put points into buckets
	for (size_t i = 0; i < points.size(); i++)
	{
		_points[i] = points[i];
		size_t key = getHashKeyFromPosition(points[i]);
		_buckets[key].push_back(i);
	}
}

Point3I PointHashGridSearcher3::getBucketIndex(const Vector3D& position) const {
	Point3I bucketIndex;
	bucketIndex.x = static_cast<size_t>(std::floor(position[0] / _gridSpacing));
	bucketIndex.y = static_cast<size_t>(std::floor(position[1] / _gridSpacing));
	bucketIndex.z = static_cast<size_t>(std::floor(position[2] / _gridSpacing));
	return bucketIndex;
}

size_t PointHashGridSearcher3::getHashKeyFromPosition(const Vector3D& position) const {
	Point3I bucketIndex = getBucketIndex(position);
	return getHashKeyFromBucketIndex(bucketIndex);
}

size_t PointHashGridSearcher3::getHashKeyFromBucketIndex(const Point3I& bucketIndex) const {
	Point3I wrappedIndex = bucketIndex;
	wrappedIndex.x = bucketIndex.x % _resolution.x;
	wrappedIndex.y = bucketIndex.y % _resolution.y;
	wrappedIndex.z = bucketIndex.z % _resolution.z;
	if (wrappedIndex.x < 0) {
		wrappedIndex.x += _resolution.x;
	}
	if (wrappedIndex.y < 0) {
		wrappedIndex.y += _resolution.y;
	}
	if (wrappedIndex.z < 0) {
		wrappedIndex.z += _resolution.z;
	}
	return static_cast<size_t>((wrappedIndex.z * _resolution.y + wrappedIndex.y) * _resolution.x+ wrappedIndex.x);
}

void PointHashGridSearcher3::forEachNearbyPoint(const Vector3D& origin, double radius, const ForEachNearbyPointFunc& callback) const {
	if (_buckets.empty()) {
		return;
	}

	size_t nearbyKeys[8];
	getNearbyKeys(origin, nearbyKeys);

	const double queryRadiusSquared = radius * radius;

	for (int i = 0; i < 8; i++) {
		const auto& bucket = _buckets[nearbyKeys[i]];
		size_t numberOfPointsInBucket = bucket.size();

		for (size_t j = 0; j < numberOfPointsInBucket; ++j) {
			size_t pointIndex = bucket[j];
			double rSquared = dist2(_points[pointIndex], origin);
			if (rSquared <= queryRadiusSquared) {
				callback(pointIndex, _points[pointIndex]);
			}
		}
	}

}

void PointHashGridSearcher3::getNearbyKeys(
    const Vector3D& position,
    size_t* nearbyKeys) const {
    Point3I originIndex = getBucketIndex(position), nearbyBucketIndices[8];

    for (int i = 0; i < 8; i++) {
        nearbyBucketIndices[i] = originIndex;
    }

    if ((originIndex.x + 0.5f) * _gridSpacing <= position[0]) {
        nearbyBucketIndices[4].x += 1;
        nearbyBucketIndices[5].x += 1;
        nearbyBucketIndices[6].x += 1;
        nearbyBucketIndices[7].x += 1;
    } else {
        nearbyBucketIndices[4].x -= 1;
        nearbyBucketIndices[5].x -= 1;
        nearbyBucketIndices[6].x -= 1;
        nearbyBucketIndices[7].x -= 1;
    }

    if ((originIndex.y + 0.5f) * _gridSpacing <= position[1]) {
        nearbyBucketIndices[2].y += 1;
        nearbyBucketIndices[3].y += 1;
        nearbyBucketIndices[6].y += 1;
        nearbyBucketIndices[7].y += 1;
    } else {
        nearbyBucketIndices[2].y -= 1;
        nearbyBucketIndices[3].y -= 1;
        nearbyBucketIndices[6].y -= 1;
        nearbyBucketIndices[7].y -= 1;
    }

    if ((originIndex.z + 0.5f) * _gridSpacing <= position[2]) {
        nearbyBucketIndices[1].z += 1;
        nearbyBucketIndices[3].z += 1;
        nearbyBucketIndices[5].z += 1;
        nearbyBucketIndices[7].z += 1;
    } else {
        nearbyBucketIndices[1].z -= 1;
        nearbyBucketIndices[3].z -= 1;
        nearbyBucketIndices[5].z -= 1;
        nearbyBucketIndices[7].z -= 1;
    }

    for (int i = 0; i < 8; i++) {
        nearbyKeys[i] = getHashKeyFromBucketIndex(nearbyBucketIndices[i]);
    }
}

