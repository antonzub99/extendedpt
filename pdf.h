#ifndef PDF_H
#define PDF_H

#include "utils.h"
#include "onb.h"

inline vec3 random_cosine_direction() {
	auto r1 = random_double();
	auto r2 = random_double();
	auto z = sqrt(1 - r2);

	auto phi = 2 * PI * r1;
	auto x = cos(phi) * sqrt(r2);
	auto y = sin(phi) * sqrt(r2);

	return vec3(x, y, z);
}

inline vec3 random_to_sphere(double radius, double distance_squared) {
	auto r1 = random_double();
	auto r2 = random_double();
	auto z = 1 + r2 * (sqrt(1 - radius * radius / distance_squared) - 1);

	auto phi = 2 * PI * r1;
	auto x = cos(phi) * sqrt(1 - z * z);
	auto y = sin(phi) * sqrt(1 - z * z);

	return vec3(x, y, z);
}

inline vec3 random_to_lobe(double shine) {
	auto r1 = random_double();
	auto r2 = random_double();
	auto z = pow(r2, 1 / (shine + 1));

	auto phi = 2 * PI * r1;
	auto x = cos(phi) * sqrt(1 - pow(r2 * r2, 1 / (shine + 1)));
	auto y = sin(phi) * sqrt(1 - pow(r2 * r2, 1 / (shine + 1)));

	return vec3(x, y, z);
}

class pdf {
public:
	virtual ~pdf() {}

	virtual double value(const vec3& direction) const = 0;
	//virtual double value(const vec3& direction, const vec3& reflected) const = 0;
	virtual vec3 generate() const = 0;
};

class cosine_pdf : public pdf{
public:
	cosine_pdf(const vec3& w) { uvw.build_from_w(w); }

	virtual double value(const vec3& direction) const override {
		auto cosine = dot(unit_vector(direction), uvw.w());
		return (cosine <= 0) ? 0 : cosine / PI;
	}

	virtual vec3 generate() const override {
		return uvw.local(random_cosine_direction());
	}

public:
	onb uvw;
};

class phong_pdf : public pdf {
public:
	phong_pdf(const vec3& w, double shine) { 
		uvw.build_from_w(w); 
		shininess = shine;
	}

	virtual double value(const vec3& direction) const override {
		auto cosine = dot(unit_vector(direction), uvw.w());
		auto factor = cosine < 0 ? 0 : pow(cosine, shininess);
		return (shininess + 1) * factor / (2 * PI);
	}

	virtual vec3 generate() const override {
		return uvw.local(random_to_lobe(shininess));
	}

public:
	onb uvw;
	double shininess;
};

class hittable_pdf : public pdf {
public: 
	hittable_pdf(shared_ptr<hittable> p, const point3& origin) : ptr(p), o(origin) {}

	virtual double value(const vec3& direction) const override {
		return ptr->pdf_value(o, direction);
	}

	virtual vec3 generate() const override {
		return ptr->random(o);
	}

public:
	point3 o;
	shared_ptr<hittable> ptr;
};

#endif