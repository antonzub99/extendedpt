#ifndef RNG_H
#define RNG_H

#include <cmath>
#include <cstdlib>
#include <limits>

inline double random_double() {
	return rand() / (RAND_MAX + 1.0);
}

inline double random_double(double min, double max) {
	return min + (max - min) * random_double();
}

inline int random_int(int min, int max) {
	return static_cast<int>(random_double(min, max + 1));
}

#endif