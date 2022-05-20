#ifndef UTILS_H
#define UTILS_H

#include "rng.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>

using std::shared_ptr;
using std::make_shared;
using std::sqrt;
using std::vector;

const double infty = std::numeric_limits<double>::infinity();
const double PI = 3.1415926535897932385;


inline int bisect(std::vector<double>& cum_weights, double& prob) {
	auto right = cum_weights.size() - 1;
	auto left = 0;

	while (left < right) {
		auto mid = left + (right - left) / 2;
		if (prob < cum_weights[mid]) right = mid;
		else left = mid + 1;
	}
	return left;
}


inline double max(double a, double b) {
	return a < b ? b : a;
}

inline double deg_to_rad(double deg) {
	return deg * PI / 180.0;
}

inline double clamp(double x, double min, double max) {
	if (x < min) return min;
	if (x > max) return max;
	return x;
}


#include "ray.h"


#endif