#ifndef SPHERE_H
#define SPHERE_H

#include "utils.h"
#include "hittable.h"
#include "pdf.h"
#include "onb.h"

class sphere : public hittable {
public:
	sphere() {}
	sphere(point3 center, double r, shared_ptr<material> m) 
		: center(center), radius(r), mat_ptr(m) {};

	virtual bool hit(
		const ray& r, double t_min, double t_max, hit_record& rec
	) const override;
	virtual double pdf_value(const point3& o, const vec3& v) const override;
	virtual vec3 random(const point3& o) const override;

public:
	point3 center;
	double radius;
	shared_ptr<material> mat_ptr;

private:
	static void get_sphere_uv(const point3& p, double& u, double& v) {
		auto theta = acos(-p.y());
		auto phi = atan2(-p.z(), p.x()) + PI;

		u = phi / (2 * PI);
		v = theta / PI;
	}
};

bool sphere::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
	//solution of quadratic equation to find intersections
	//(if there are any) of ray with the sphere
	vec3 oc = r.origin() - center;
	auto a = r.direction().length_squared();
	auto half_b = dot(oc, r.direction());
	auto c = oc.length_squared() - radius * radius;

	auto discriminant = half_b * half_b - a * c;
	if (discriminant < 0) return false;
	auto sqrtd = sqrt(discriminant);

	//find the nearest root that lies in acceptable range
	auto root = (-half_b - sqrtd) / a;
	if (root < t_min || t_max < root) {
		root = (-half_b + sqrtd) / a;
		if (root < t_min || t_max < root)
			return false;
	}

	rec.t = root;
	rec.p = r.at(rec.t);
	vec3 outward_normal = (rec.p - center) / radius;
	rec.set_face_normal(r, outward_normal);
	get_sphere_uv(outward_normal, rec.u, rec.v);
	rec.mat_ptr = mat_ptr;

	return true;
}

double sphere::pdf_value(const point3& o, const vec3& v) const {
	hit_record rec;
	if (!this->hit(ray(o, v), 0.001, infty, rec))
		return 0;

	auto cos_theta_max = sqrt(1 - radius * radius / (center - o).length_squared());
	auto solid_angle = 2 * PI * (1 - cos_theta_max);

	return 1 / solid_angle;
}

vec3 sphere::random(const point3& o) const {
	vec3 direction = center - o;
	auto distance_squared = direction.length_squared();
	onb uvw;
	uvw.build_from_w(direction);
	return uvw.local(random_to_sphere(radius, distance_squared));
}


#endif
