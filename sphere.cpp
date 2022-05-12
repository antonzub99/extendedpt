/*
#include "sphere.h"
#include "utils.h"
#include <cmath>

inline void get_sphere_uv(const point3& p, double& u, double& v) {
	auto theta = acos(-p.y());
	auto phi = atan2(-p.z(), p.x()) + PI;

	u = phi / (2 * PI);
	v = theta / PI;
}
*/

