#ifndef VEC_H
#define VEC_H

#include <cassert>
#include <cmath>
#include <iostream>
#include "util.h"

// Defines a thin wrapper around fixed size C-style arrays, using template parameters,
// which is useful for dealing with vectors of different dimensions.
// For example, float[3] is equivalent to Vec<3,float>.
// Entries in the vector are accessed with the overloaded [] operator, so
// for example if x is a Vec<3,float>, then the middle entry is x[1].
// For convenience, there are a number of typedefs for abbreviation:
//   Vec<3,float> -> Vec3f
//   Vec<2,int>   -> Vec2i
// and so on.
// Arithmetic operators are appropriately overloaded, and functions are defined
// for additional operations (such as dot-products, norms, cross-products, etc.)

template<unsigned int N, class T>
struct Vector
{
	T v[N];

	Vector<N, T>(void)
	{}

	Vector<N, T>(T value_for_all)
	{
		for (unsigned int i = 0; i < N; ++i) v[i] = value_for_all;
	}

	template<class S>
	Vector<N, T>(const S* source)
	{
		for (unsigned int i = 0; i < N; ++i) v[i] = (T)source[i];
	}

	template <class S>
	explicit Vector<N, T>(const Vector<N, S>& source)
	{
		for (unsigned int i = 0; i < N; ++i) v[i] = (T)source[i];
	}

	Vector<N, T>(T v0, T v1)
	{
		assert(N == 2);
		v[0] = v0; v[1] = v1;
	}

	Vector<N, T>(T v0, T v1, T v2)
	{
		assert(N == 3);
		v[0] = v0; v[1] = v1; v[2] = v2;
	}

	Vector<N, T>(T v0, T v1, T v2, T v3)
	{
		assert(N == 4);
		v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3;
	}

	Vector<N, T>(T v0, T v1, T v2, T v3, T v4)
	{
		assert(N == 5);
		v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3; v[4] = v4;
	}

	Vector<N, T>(T v0, T v1, T v2, T v3, T v4, T v5)
	{
		assert(N == 6);
		v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3; v[4] = v4; v[5] = v5;
	}

	T& operator[](int index)
	{
		assert(0 <= index && (unsigned int)index < N);
		return v[index];
	}

	const T& operator[](int index) const
	{
		assert(0 <= index && (unsigned int)index < N);
		return v[index];
	}

	operator bool(void) const
	{
		for (unsigned int i = 0; i < N; ++i) if (v[i]) return true;
		return false;
	}

	Vector<N, T> operator+=(const Vector<N, T>& w)
	{
		for (unsigned int i = 0; i < N; ++i) v[i] += w[i];
		return *this;
	}

	Vector<N, T> operator+(const Vector<N, T>& w) const
	{
		Vector<N, T> sum(*this);
		sum += w;
		return sum;
	}

	Vector<N, T> operator-=(const Vector<N, T>& w)
	{
		for (unsigned int i = 0; i < N; ++i) v[i] -= w[i];
		return *this;
	}

	Vector<N, T> operator-(void) const // unary minus
	{
		Vector<N, T> negative;
		for (unsigned int i = 0; i < N; ++i) negative.v[i] = -v[i];
		return negative;
	}

	Vector<N, T> operator-(const Vector<N, T>& w) const // (binary) subtraction
	{
		Vector<N, T> diff(*this);
		diff -= w;
		return diff;
	}

	Vector<N, T> operator*=(T a)
	{
		for (unsigned int i = 0; i < N; ++i) v[i] *= a;
		return *this;
	}

	Vector<N, T> operator*(T a) const
	{
		Vector<N, T> w(*this);
		w *= a;
		return w;
	}

	Vector<N, T> operator*(const Vector<N, T>& w) const
	{
		Vector<N, T> componentwise_product;
		for (unsigned int i = 0; i < N; ++i) componentwise_product[i] = v[i] * w.v[i];
		return componentwise_product;
	}

	Vector<N, T> operator/=(T a)
	{
		for (unsigned int i = 0; i < N; ++i) v[i] /= a;
		return *this;
	}

	Vector<N, T> operator/(T a) const
	{
		Vector<N, T> w(*this);
		w /= a;
		return w;
	}
};

typedef Vector<2, double>         Vector2D;
typedef Vector<2, float>          Vector2F;
typedef Vector<2, int>            Vector2I;
typedef Vector<2, unsigned int>   Vector2UI;
typedef Vector<2, short>          Vector2S;
typedef Vector<2, unsigned short> Vector2US;
typedef Vector<2, char>           Vector2C;
typedef Vector<2, unsigned char>  Vector2UC;

typedef Vector<3, double>         Vector3D;
typedef Vector<3, float>          Vector3F;
typedef Vector<3, int>            Vector3I;
typedef Vector<3, unsigned int>   Vector3UI;
typedef Vector<3, short>          Vector3S;
typedef Vector<3, unsigned short> Vector3US;
typedef Vector<3, char>           Vector3C;
typedef Vector<3, unsigned char>  Vector3UC;

typedef Vector<4, double>         Vector4D;
typedef Vector<4, float>          Vector4F;
typedef Vector<4, int>            Vector4I;
typedef Vector<4, unsigned int>   Vector4UI;
typedef Vector<4, short>          Vector4S;
typedef Vector<4, unsigned short> Vector4US;
typedef Vector<4, char>           Vector4C;
typedef Vector<4, unsigned char>  Vector4UC;

typedef Vector<6, double>         Vector6D;
typedef Vector<6, float>          Vector6F;
typedef Vector<6, unsigned int>   Vector6UI;
typedef Vector<6, int>            Vector6I;
typedef Vector<6, short>          Vector6S;
typedef Vector<6, unsigned short> Vector6US;
typedef Vector<6, char>           Vector6C;
typedef Vector<6, unsigned char>  Vector6UC;


template<unsigned int N, class T>
T mag2(const Vector<N, T>& a)
{
	T l = sqr(a.v[0]);
	for (unsigned int i = 1; i < N; ++i) l += sqr(a.v[i]);
	return l;
}

template<unsigned int N, class T>
T mag(const Vector<N, T>& a)
{
	return sqrt(mag2(a));
}

template<unsigned int N, class T>
inline T dist2(const Vector<N, T>& a, const Vector<N, T>& b)
{
	T d = sqr(a.v[0] - b.v[0]);
	for (unsigned int i = 1; i < N; ++i) d += sqr(a.v[i] - b.v[i]);
	return d;
}

template<unsigned int N, class T>
inline T dist(const Vector<N, T>& a, const Vector<N, T>& b)
{
	return std::sqrt(dist2(a, b));
}

template<unsigned int N, class T>
inline void normalize(Vector<N, T>& a)
{
	a /= mag(a);
}

template<unsigned int N, class T>
inline Vector<N, T> normalized(const Vector<N, T>& a)
{
	return a / mag(a);
}

template<unsigned int N, class T>
inline T infnorm(const Vector<N, T>& a)
{
	T d = std::fabs(a.v[0]);
	for (unsigned int i = 1; i < N; ++i) d = max(std::fabs(a.v[i]), d);
	return d;
}

template<unsigned int N, class T>
void zero(Vector<N, T>& a)
{
	for (unsigned int i = 0; i < N; ++i)
		a.v[i] = 0;
}

template<unsigned int N, class T>
std::ostream& operator<<(std::ostream& out, const Vector<N, T>& v)
{
	out << v.v[0];
	for (unsigned int i = 1; i < N; ++i)
		out << ' ' << v.v[i];
	return out;
}

template<unsigned int N, class T>
std::istream& operator>>(std::istream& in, Vector<N, T>& v)
{
	in >> v.v[0];
	for (unsigned int i = 1; i < N; ++i)
		in >> v.v[i];
	return in;
}

template<unsigned int N, class T>
inline bool operator==(const Vector<N, T>& a, const Vector<N, T>& b)
{
	bool t = (a.v[0] == b.v[0]);
	unsigned int i = 1;
	while (i < N && t) {
		t = t && (a.v[i] == b.v[i]);
		++i;
	}
	return t;
}

template<unsigned int N, class T>
inline bool operator!=(const Vector<N, T>& a, const Vector<N, T>& b)
{
	bool t = (a.v[0] != b.v[0]);
	unsigned int i = 1;
	while (i < N && !t) {
		t = t || (a.v[i] != b.v[i]);
		++i;
	}
	return t;
}

template<unsigned int N, class T>
inline Vector<N, T> operator*(T a, const Vector<N, T>& v)
{
	Vector<N, T> w(v);
	w *= a;
	return w;
}

template<unsigned int N, class T>
inline T min(const Vector<N, T>& a)
{
	T m = a.v[0];
	for (unsigned int i = 1; i < N; ++i) if (a.v[i] < m) m = a.v[i];
	return m;
}

template<unsigned int N, class T>
inline Vector<N, T> min_union(const Vector<N, T>& a, const Vector<N, T>& b)
{
	Vector<N, T> m;
	for (unsigned int i = 0; i < N; ++i) (a.v[i] < b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i];
	return m;
}

template<unsigned int N, class T>
inline Vector<N, T> max_union(const Vector<N, T>& a, const Vector<N, T>& b)
{
	Vector<N, T> m;
	for (unsigned int i = 0; i < N; ++i) (a.v[i] > b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i];
	return m;
}

template<unsigned int N, class T>
inline T max(const Vector<N, T>& a)
{
	T m = a.v[0];
	for (unsigned int i = 1; i < N; ++i) if (a.v[i] > m) m = a.v[i];
	return m;
}

template<unsigned int N, class T>
inline T dot(const Vector<N, T>& a, const Vector<N, T>& b)
{
	T d = a.v[0] * b.v[0];
	for (unsigned int i = 1; i < N; ++i) d += a.v[i] * b.v[i];
	return d;
}

template<class T>
inline Vector<2, T> rotate(const Vector<2, T>& a, float angle)
{
	T c = cos(angle);
	T s = sin(angle);
	return Vector<2, T>(c * a[0] - s * a[1], s * a[0] + c * a[1]); // counter-clockwise rotation
}

template<class T>
inline Vector<2, T> perp(const Vector<2, T>& a)
{
	return Vector<2, T>(-a.v[1], a.v[0]);
} // counter-clockwise rotation by 90 degrees

template<class T>
inline T cross(const Vector<2, T>& a, const Vector<2, T>& b)
{
	return a.v[0] * b.v[1] - a.v[1] * b.v[0];
}

template<class T>
inline Vector<3, T> cross(const Vector<3, T>& a, const Vector<3, T>& b)
{
	return Vector<3, T>(a.v[1] * b.v[2] - a.v[2] * b.v[1], a.v[2] * b.v[0] - a.v[0] * b.v[2], a.v[0] * b.v[1] - a.v[1] * b.v[0]);
}

template<class T>
inline T triple(const Vector<3, T>& a, const Vector<3, T>& b, const Vector<3, T>& c)
{
	return a.v[0] * (b.v[1] * c.v[2] - b.v[2] * c.v[1])
		+ a.v[1] * (b.v[2] * c.v[0] - b.v[0] * c.v[2])
		+ a.v[2] * (b.v[0] * c.v[1] - b.v[1] * c.v[0]);
}

template<unsigned int N, class T>
inline unsigned int hash(const Vector<N, T>& a)
{
	unsigned int h = a.v[0];
	for (unsigned int i = 1; i < N; ++i)
		h = hash(h ^ a.v[i]);
	return h;
}

template<unsigned int N, class T>
inline void assign(const Vector<N, T>& a, T& a0, T& a1)
{
	assert(N == 2);
	a0 = a.v[0]; a1 = a.v[1];
}

template<unsigned int N, class T>
inline void assign(const Vector<N, T>& a, T& a0, T& a1, T& a2)
{
	assert(N == 3);
	a0 = a.v[0]; a1 = a.v[1]; a2 = a.v[2];
}

template<unsigned int N, class T>
inline void assign(const Vector<N, T>& a, T& a0, T& a1, T& a2, T& a3)
{
	assert(N == 4);
	a0 = a.v[0]; a1 = a.v[1]; a2 = a.v[2]; a3 = a.v[3];
}

template<unsigned int N, class T>
inline void assign(const Vector<N, T>& a, T& a0, T& a1, T& a2, T& a3, T& a4, T& a5)
{
	assert(N == 6);
	a0 = a.v[0]; a1 = a.v[1]; a2 = a.v[2]; a3 = a.v[3]; a4 = a.v[4]; a5 = a.v[5];
}

template<unsigned int N, class T>
inline Vector<N, int> round(const Vector<N, T>& a)
{
	Vector<N, int> rounded;
	for (unsigned int i = 0; i < N; ++i)
		rounded.v[i] = lround(a.v[i]);
	return rounded;
}

template<unsigned int N, class T>
inline Vector<N, int> floor(const Vector<N, T>& a)
{
	Vector<N, int> rounded;
	for (unsigned int i = 0; i < N; ++i)
		rounded.v[i] = (int)floor(a.v[i]);
	return rounded;
}

template<unsigned int N, class T>
inline Vector<N, int> ceil(const Vector<N, T>& a)
{
	Vector<N, int> rounded;
	for (unsigned int i = 0; i < N; ++i)
		rounded.v[i] = (int)ceil(a.v[i]);
	return rounded;
}

template<unsigned int N, class T>
inline Vector<N, T> fabs(const Vector<N, T>& a)
{
	Vector<N, T> result;
	for (unsigned int i = 0; i < N; ++i)
		result.v[i] = fabs(a.v[i]);
	return result;
}

template<unsigned int N, class T>
inline void minmax(const Vector<N, T>& x0, const Vector<N, T>& x1, Vector<N, T>& xmin, Vector<N, T>& xmax)
{
	for (unsigned int i = 0; i < N; ++i)
		minmax(x0.v[i], x1.v[i], xmin.v[i], xmax.v[i]);
}

template<unsigned int N, class T>
inline void minmax(const Vector<N, T>& x0, const Vector<N, T>& x1, const Vector<N, T>& x2, Vector<N, T>& xmin, Vector<N, T>& xmax)
{
	for (unsigned int i = 0; i < N; ++i)
		minmax(x0.v[i], x1.v[i], x2.v[i], xmin.v[i], xmax.v[i]);
}

template<unsigned int N, class T>
inline void minmax(const Vector<N, T>& x0, const Vector<N, T>& x1, const Vector<N, T>& x2, const Vector<N, T>& x3,
	Vector<N, T>& xmin, Vector<N, T>& xmax)
{
	for (unsigned int i = 0; i < N; ++i)
		minmax(x0.v[i], x1.v[i], x2.v[i], x3.v[i], xmin.v[i], xmax.v[i]);
}

template<unsigned int N, class T>
inline void minmax(const Vector<N, T>& x0, const Vector<N, T>& x1, const Vector<N, T>& x2, const Vector<N, T>& x3, const Vector<N, T>& x4,
	Vector<N, T>& xmin, Vector<N, T>& xmax)
{
	for (unsigned int i = 0; i < N; ++i)
		minmax(x0.v[i], x1.v[i], x2.v[i], x3.v[i], x4.v[i], xmin.v[i], xmax.v[i]);
}

template<unsigned int N, class T>
inline void minmax(const Vector<N, T>& x0, const Vector<N, T>& x1, const Vector<N, T>& x2, const Vector<N, T>& x3, const Vector<N, T>& x4,
	const Vector<N, T>& x5, Vector<N, T>& xmin, Vector<N, T>& xmax)
{
	for (unsigned int i = 0; i < N; ++i)
		minmax(x0.v[i], x1.v[i], x2.v[i], x3.v[i], x4.v[i], x5.v[i], xmin.v[i], xmax.v[i]);
}

template<unsigned int N, class T>
inline void update_minmax(const Vector<N, T>& x, Vector<N, T>& xmin, Vector<N, T>& xmax)
{
	for (unsigned int i = 0; i < N; ++i) update_minmax(x[i], xmin[i], xmax[i]);
}

#endif
