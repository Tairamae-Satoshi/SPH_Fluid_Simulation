#pragma once

template <typename T>
class Point3 {
public:
	T x;
	T y;
	T z;

	Point3() {};
	Point3(T x, T y, T z):x(x), y(y), z(z) {};
	Point3(const Point3& p):x(p.x), y(p.y), z(p.z) {};
};

typedef Point3<int> Point3I;
typedef Point3<double> Point3D;