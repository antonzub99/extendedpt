#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "utils.h"
#include "hittable_list.h"
#include "material.h"

struct recursion_record {
	int depth;
	int start_roulette;
	int max_depth;
	double prob;
	color acc_color;
};

class Integrator {
public:
	virtual ~Integrator() {}

	virtual color Li(
		const ray& r,
		const hittable& world
	) const {
		return color();
	}

	virtual color Li(
		const ray& r,
		const hittable& world,
		shared_ptr<hittable>& lights,
		recursion_record& next_ray
	) const {
		return color();
	}

	virtual color Li(
		const ray& r,
		const hittable& world,
		recursion_record& next_ray
	) const {
		return color();
	}
};

class NormalIntegrator : public Integrator {
public:
	virtual color Li(const ray& r, const hittable& world) const override {
		hit_record rec;

		if (!world.hit(r, 0.001, infty, rec)) return color();
		else return rec.normal.abs();
	}

};

class PathTracerMaterials : public Integrator {
public:
	virtual color Li(const ray& r, const hittable& world, shared_ptr<hittable>& lights, recursion_record& next_ray) const override {
		hit_record rec;

		next_ray.depth += 1;
		if (next_ray.depth > next_ray.max_depth) {
			return color();
		};

		auto cur_color = next_ray.acc_color;
		
		if (!world.hit(r, 0.001, infty, rec)) return color();

		scatter_record srec;
		color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);
		
		if (next_ray.depth > next_ray.start_roulette) {
			next_ray.prob = clamp(max_comp(cur_color), 0.000001, 0.99999);
		}

		if (random_double() >= next_ray.prob) {
			return emitted;
		}

		if (!rec.mat_ptr->sample(r.direction(), rec, srec)) return emitted;

		if (srec.is_specular) {
			auto attenuation = rec.mat_ptr->eval(r.direction(), rec, srec.specular_ray.direction());
			next_ray.acc_color = cur_color * attenuation;
			return attenuation * PathTracerMaterials::Li(srec.specular_ray, world, lights, next_ray) / next_ray.prob;
		}

		auto irradiance = rec.mat_ptr->eval(r.direction(), rec, srec.scattered);
		auto pdf_val = rec.mat_ptr->pdf(r.direction(), rec, srec);
		cur_color = cur_color * irradiance / (pdf_val * next_ray.prob);
		
		auto scattered_ray = ray(rec.p, srec.scattered);
		next_ray.acc_color = cur_color;
		return emitted + irradiance * PathTracerMaterials::Li(scattered_ray, world, lights, next_ray) / (pdf_val * next_ray.prob);
	}
};

class MISIntegrator : public Integrator {
public:
	virtual color Li(const ray& r, const hittable& world, shared_ptr<hittable>& lights, recursion_record& next_ray) const override {
		hit_record rec;

		next_ray.depth += 1;
		if (next_ray.depth > next_ray.max_depth) {
			return color();
		};

		auto cur_color = next_ray.acc_color;

		if (!world.hit(r, 0.001, infty, rec)) return color();

		scatter_record srec;
		color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);

		if (next_ray.depth > next_ray.start_roulette) {
			next_ray.prob = clamp(max_comp(cur_color), 0.000001, 0.99999);
		}

		if (random_double() >= next_ray.prob) {
			return emitted;
		}

		if (!rec.mat_ptr->sample(r.direction(), rec, srec)) return emitted;

		if (srec.is_specular) {
			auto attenuation = rec.mat_ptr->eval(r.direction(), rec, srec.specular_ray.direction());
			next_ray.acc_color = cur_color * attenuation;
			return attenuation * this->Li(srec.specular_ray, world, lights, next_ray) / next_ray.prob;
		}

		auto coin = random_double();

		auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
		auto light_dir = light_ptr->generate();
		auto light_ray = ray(rec.p, light_dir);
		auto pdf_light = light_ptr->value(light_ray.direction());

		auto pdf_mat = rec.mat_ptr->pdf(r.direction(), rec, srec);

		if (coin < 0.5) {
			auto light_irr = rec.mat_ptr->eval(r.direction(), rec, light_dir);
			auto weight = pdf_light / (pdf_light + pdf_mat);
			//next_ray.depth = next_ray.max_depth;
			return emitted + weight * light_irr * MISIntegrator::Li(light_ray, world, lights, next_ray) / (pdf_light * next_ray.prob);
		}
		else {
			auto mat_ray = ray(rec.p, srec.scattered);
			auto mat_irr = rec.mat_ptr->eval(r.direction(), rec, srec.scattered);
			auto weight = pdf_mat / (pdf_light + pdf_mat);
			cur_color = cur_color * mat_irr / (pdf_mat * next_ray.prob);
			next_ray.acc_color = cur_color;
			return emitted + weight * mat_irr * MISIntegrator::Li(mat_ray, world, lights, next_ray) / (pdf_mat * next_ray.prob);
		}	
	}
};

class PathMISIntegrator : public Integrator {
public:
	virtual color Li(const ray& r, const hittable& world, shared_ptr<hittable>& lights, recursion_record& next_ray) const override {
		hit_record rec;

		next_ray.depth += 1;
		if (next_ray.depth > next_ray.max_depth) {
			return color();
		};

		auto cur_color = next_ray.acc_color;

		if (!world.hit(r, 0.001, infty, rec)) return color();

		scatter_record srec;
		color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);

		if (next_ray.depth > next_ray.start_roulette) {
			next_ray.prob = clamp(max_comp(cur_color), 0.000001, 0.99999);
		}

		if (random_double() >= next_ray.prob) {
			return emitted;
		}

		if (!rec.mat_ptr->sample(r.direction(), rec, srec)) return emitted;

		if (srec.is_specular) {
			auto attenuation = rec.mat_ptr->eval(r.direction(), rec, srec.specular_ray.direction());
			next_ray.acc_color = cur_color * attenuation;
			return attenuation * PathMISIntegrator::Li(srec.specular_ray, world, lights, next_ray) / next_ray.prob;
		}

		auto coin = random_double();

		auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
		auto light_dir = light_ptr->generate();
		auto light_ray = ray(rec.p, light_dir);
		auto pdf_light = light_ptr->value(light_ray.direction());

		auto pdf_mat = rec.mat_ptr->pdf(r.direction(), rec, srec);
		auto mat_ray = ray(rec.p, srec.scattered);

		if (coin < 0.5) {
			auto light_irr = rec.mat_ptr->eval(r.direction(), rec, light_dir);
			auto weight = pdf_light / (pdf_light + pdf_mat);
			//next_ray.depth = next_ray.max_depth;
			return emitted + weight * light_irr / pdf_light;
		}
		else {
			auto mat_irr = rec.mat_ptr->eval(r.direction(), rec, srec.scattered);
			auto weight = pdf_mat / (pdf_light + pdf_mat);
			cur_color = cur_color * mat_irr / (pdf_mat * next_ray.prob);
			next_ray.acc_color = cur_color;
			return emitted + weight * mat_irr * PathMISIntegrator::Li(mat_ray, world, lights, next_ray) / (pdf_mat * next_ray.prob);
		}
	}
};


#endif
