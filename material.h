#ifndef MATERIAL_H
#define MATERIAL_H

#include <cmath>
#include "utils.h"
#include "texture.h"
#include "pdf.h"


//struct hit_record;

struct scatter_record {
	ray specular_ray;
	vec3 scattered;
	bool is_specular;
	//color attenuation;
	shared_ptr<pdf> pdf_ptr;
};


class material {
public:
	//for light emissive materials
	virtual color emitted(
		const ray& r_in, const hit_record& rec, double u, double v, const point3& p
	) const {
		return color(0, 0, 0);
	}

	//evaluate the material response value given ray sample
	//basically brdf * cosine
	virtual color eval(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		return color(0, 0, 0);
	}
	
	//sample a scattered ray according to brdf pdf
	virtual bool sample(
		const vec3& r_in, const hit_record& rec, scatter_record& srec
	) const {
		return false;
	};

	//evaluate pdf given sampled ray
	virtual double pdf(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		return 0;
	}

	/*
	the same as above but with scatter record
	we actually store both pointer to the sampling pdf and sampled direction 
	in the srec
	but it works only if we call pdf after filling srec in sample method
	*/
	virtual double pdf(
		const vec3& r_in, const hit_record& rec, const scatter_record& srec
	) const {
		return 0;
	}
};

class lambertian : public material {
public:
	lambertian(const color& a) : albedo(make_shared<solid_color>(a)) {}
	lambertian(shared_ptr<texture> a) : albedo(a) {}

	virtual bool sample(
		const vec3& r_in, const hit_record& rec, scatter_record& srec
	) const override {
		srec.is_specular = false;
		//srec.pdf_ptr = make_shared<cosine_pdf>(rec.normal);
		srec.pdf_ptr = make_shared<hemisphere_pdf>(rec.normal);
		srec.scattered = srec.pdf_ptr->generate();
		return true;
	}

	color eval(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		auto scatter_dir = unit_vector(scattered);
		auto cosine = max(dot(rec.normal, scatter_dir), 0.0);
		return albedo->value(rec.u, rec.v, rec.p) * cosine / PI;
	}

	double pdf(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		auto scatter_dir = unit_vector(scattered);
		auto cosine = max(dot(rec.normal, scatter_dir), 0.0);
		return cosine / PI;
	}

	double pdf(
		const vec3& r_in, const hit_record& rec, const scatter_record& srec
	) const {
		return srec.pdf_ptr->value(srec.scattered);
	}
public: 
	shared_ptr<texture> albedo;
};

class phong : public material {
public:
	phong(const color& a, double shine) : albedo(make_shared<solid_color>(a)), shininess(shine) {}
	phong(shared_ptr<texture> a, double shine) : albedo(a), shininess(shine) {}

	virtual bool sample(
		const vec3& r_in, const hit_record& rec, scatter_record& srec
	) const override {
		auto mirror = reflect(unit_vector(r_in), rec.normal);
		srec.is_specular = false;
		//srec.pdf_ptr = make_shared<phong_pdf>(mirror, shininess);
		srec.pdf_ptr = make_shared<hemisphere_pdf>(rec.normal);
		srec.scattered = srec.pdf_ptr->generate();
		return true;
	}

	color eval(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		auto mirror = reflect(unit_vector(r_in), rec.normal);
		auto brdf = pow(max(dot(unit_vector(scattered), mirror), 0.0), shininess);
		return albedo->value(rec.u, rec.v, rec.p) * brdf * (shininess + 1) / (2 * PI);
	}

	double pdf(
		const vec3& r_in, const hit_record& rec, const scatter_record& srec
	) const {
		return srec.pdf_ptr->value(srec.scattered);
	}
	
public:
	shared_ptr<texture> albedo;
	double shininess;
};


class blinn_phong : public material {
public:
	blinn_phong(const color& a, double shine) : albedo(make_shared<solid_color>(a)), shininess(shine) {}
	blinn_phong(shared_ptr<texture> a, double shine) : albedo(a), shininess(shine) {}
	

	virtual bool sample(
		const vec3& r_in, const hit_record& rec, scatter_record& srec
	) const override {
		srec.is_specular = false;
		//srec.pdf_ptr = make_shared<blinn_phong_pdf>(rec.normal, shininess);
		srec.pdf_ptr = make_shared<hemisphere_pdf>(rec.normal);
		auto new_normal = srec.pdf_ptr->generate();

		srec.scattered = reflect(unit_vector(r_in), new_normal);
		
		//if (srec.scattered.z() < 0) return false;
		return true;
	}

	color eval(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		auto randomnormal = unit_vector(-unit_vector(r_in) + unit_vector(scattered));
		auto cosine = max(dot(randomnormal, rec.normal), 0.0);
		auto normalpdf = (shininess + 1) * pow(cosine, shininess) / (2 * PI);
		auto brdf = normalpdf / (4 * dot(-r_in, randomnormal));
		return albedo->value(rec.u, rec.v, rec.p) * brdf;
	}

	double pdf(
		const vec3& r_in, const hit_record& rec, const scatter_record& srec
	) const {
		//auto randomnormal = unit_vector(-unit_vector(r_in) + unit_vector(srec.scattered));
		//auto normalpdf = srec.pdf_ptr->value(randomnormal);
		//return normalpdf / (4 * dot(-r_in, randomnormal));
		return srec.pdf_ptr->value(srec.scattered);
	}
public:
	shared_ptr<texture> albedo;
	double shininess;
};


class metal : public material {
public:
	metal(const color& a, double f) : albedo(a), fuzz(f < 1 ? f : 1) {}

	virtual bool sample(
		const vec3& r_in, const hit_record& rec, scatter_record& srec
	) const override {
		vec3 reflected = reflect(unit_vector(r_in), rec.normal);
		srec.specular_ray = ray(rec.p, reflected + fuzz * random_in_unit_sphere());
		srec.is_specular = true;
		srec.pdf_ptr = nullptr;
		return true;
	}

	color eval(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		return albedo;
	}

public:
	color albedo;
	double fuzz;
};

class dielectric : public material {
public: 
	dielectric(double index_of_refraction) : ir(index_of_refraction) {}

	virtual bool sample(
		const vec3& r_in, const hit_record& rec, scatter_record& srec
	) const override {
		srec.is_specular = true;
		srec.pdf_ptr = nullptr;
		double refraction_ratio = rec.front_face ? (1.0 / ir) : ir;

		vec3 unit_direction = unit_vector(r_in);
		double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
		double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

		bool cannot_refract = refraction_ratio * sin_theta > 1.0;
		vec3 direction;

		if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
			direction = reflect(unit_direction, rec.normal);
		else
			direction = refract(unit_direction, rec.normal, refraction_ratio);
		srec.specular_ray = ray(rec.p, direction);
		return true;
	}

	color eval(
		const vec3& r_in, const hit_record& rec, const vec3& scattered
	) const {
		return color(1.0, 1.0, 1.0);
	}

public:
	double ir;

private:
	static double reflectance(double cosine, double ref_idx) {
		auto r0 = (1 - ref_idx) / (1 + ref_idx);
		r0 = r0 * r0;
		return r0 + (1 - r0) * pow((1 - cosine), 5);
	}
};

class diffuse_light : public material {
public:
	diffuse_light(shared_ptr<texture> a) : emit(a) {}
	diffuse_light(color c) : emit(make_shared<solid_color>(c)) {}

	virtual color emitted(const ray& r_in, const hit_record& rec, double u, double v, 
		const point3& p) const override {
		if (!rec.front_face)
			return color(0, 0, 0);
		return emit->value(u, v, p);
	}

public:
	shared_ptr<texture> emit;
};

#endif
