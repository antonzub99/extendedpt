#include "renderer.h"
#include "utils.h"
#include "material.h"

#include <iostream>

enum Strategy { BRDF, LIGHT, MIS };

struct RenderSetup {
	color background;
	Strategy mode;
	int num_samples;
	int init_depth;
	color throughput;
};

hittable_list simple_scene(const point3& loc, const double& radius, const color& col) {
	hittable_list objects;

	auto material_left = make_shared<phong>(color(0.75, 0.25, 0.25), 50);
	auto material_right = make_shared<lambertian>(color(0.25, 0.25, 0.75));
	auto material_back = make_shared<lambertian>(color(0.75, 0.75, 0.75));
	//auto material_front = make_shared<lambertian>(color());
	auto material_bot = make_shared<lambertian>(color(0.75, 0.75, 0.75));
	auto material_top = make_shared<lambertian>(color(0.75, 0.75, 0.75));

	auto material_first = make_shared<dielectric>(1.5);
	auto material_second = make_shared<metal>(color(1, 1, 1), 0.2);
	auto material_third = make_shared<phong>(color(0.5, 0.2, 0.2), 2000);
	auto material_forth = make_shared<blinn_phong>(color(0.333, 0.983, 0.015), 100);

	auto difflight = make_shared<diffuse_light>(col);


	objects.add(make_shared<sphere>(point3(1e5+1, 40.8, 81.6), 1e5, material_left));
	//objects.add(make_shared<sphere>(point3(-1e5 + 99, 40.8, 81.6), 1e5, material_forth));
	objects.add(make_shared<sphere>(point3(50, 40.8, 1e5), 1e5, material_back));
	//objects.add(make_shared<sphere>(point3(50, 40.8, -1e5+170), 1e5, material_front));
	objects.add(make_shared<sphere>(point3(50, 1e5, 81.6), 1e5, material_bot));
	objects.add(make_shared<sphere>(point3(50, -1e5+81.6, 81.6), 1e5, material_top));
	objects.add(make_shared<sphere>(point3(27, 16.5, 47), 16.5, material_first));
	objects.add(make_shared<sphere>(point3(73, 16.5, 78), 16.5, material_second));
	objects.add(make_shared<sphere>(point3(51, 18.5, 90), 10.5, material_third));
	objects.add(make_shared<sphere>(point3(39, 5.5, 120), 15.5, material_forth));
	objects.add(make_shared<sphere>(loc, radius, difflight));
	return objects;
}

hittable_list simplest_scene(const point3& loc, const double& radius, const color& col) {
	hittable_list objects;

	//auto material_left = make_shared<lambertian>(color(0.75, 0.25, 0.25));
	//auto material_right = make_shared<lambertian>(color(0.25, 0.25, 0.75));
	//auto material_back = make_shared<lambertian>(color(0.75, 0.75, 0.75));
	//auto material_front = make_shared<lambertian>(color());
	auto material_bot = make_shared<lambertian>(color(0.75, 0.75, 0.75));
	auto material_top = make_shared<lambertian>(color(0.75, 0.75, 0.75));

	auto material_first = make_shared<dielectric>(1.5);
	auto material_second = make_shared<metal>(color(1, 1, 1), 0.2);
	auto material_third = make_shared<phong>(color(0.5, 0.2, 0.2), 50);
	auto material_forth = make_shared<lambertian>(color(0.3, 0.8, 0.1));

	auto difflight = make_shared<diffuse_light>(col);

	/*
	objects.add(make_shared<sphere>(point3(1e5 + 1, 40.8, 81.6), 1e5, material_left));
	objects.add(make_shared<sphere>(point3(-1e5 + 99, 40.8, 81.6), 1e5, material_right));
	objects.add(make_shared<sphere>(point3(50, 40.8, 1e5), 1e5, material_back));
	objects.add(make_shared<sphere>(point3(50, 40.8, -1e5+170), 1e5, material_front));
	*/

	objects.add(make_shared<sphere>(point3(50, 1e5, 81.6), 1e5, material_bot));
	objects.add(make_shared<sphere>(point3(50, -1e5 + 81.6, 81.6), 1e5, material_top));
	objects.add(make_shared<sphere>(point3(40, 16.5, 25), 16.5, material_first));
	objects.add(make_shared<sphere>(point3(40, 16.5, 58), 16.5, material_second));
	objects.add(make_shared<sphere>(point3(40, 18.5, 80), 10.5, material_third));
	objects.add(make_shared<sphere>(point3(40, 16.5, 130), 20.5, material_forth));
	objects.add(make_shared<sphere>(loc, radius, difflight));
	return objects;
}


color ray_color(
	const ray& r, 
	const color& background, 
	const hittable& world, 
	shared_ptr<hittable>& lights,
	int depth,
	int mode,
	const int num_samples
) {
	hit_record rec;

	if (depth < 0) return color(0, 0, 0);

	if (!world.hit(r, 0.001, infty, rec)) return background;

	scatter_record srec;
	color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);

	if (!rec.mat_ptr->sample(r.direction(), rec, srec)) return emitted;

	if (srec.is_specular) {
		auto attenuation = rec.mat_ptr->eval(r.direction(), rec, srec.specular_ray.direction());
		return attenuation * ray_color(srec.specular_ray, background, world, lights, depth - 1, mode, num_samples);
	}

	if (mode == 0) {
		auto irradiance = rec.mat_ptr->eval(r.direction(), rec, srec.scattered);
		auto pdf_val = rec.mat_ptr->pdf(r.direction(), rec, srec);
		auto scattered_ray = ray(rec.p, srec.scattered);
		return emitted + irradiance * ray_color(scattered_ray, background, world, lights, depth - 1, mode,
			num_samples) / pdf_val;
	}

	if (mode == 1) {
		auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
		auto scattered_dir = light_ptr->generate();
		auto scattered_ray = ray(rec.p, scattered_dir);

		auto irradiance = rec.mat_ptr->eval(r.direction(), rec, scattered_dir);
		auto pdf_val = light_ptr->value(scattered_ray.direction());
		return emitted + irradiance * ray_color(scattered_ray, background, world, lights, depth - 1, mode,
			num_samples) / pdf_val;
	}
	if (mode == 2) {
		auto coin = random_double();

		auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
		auto light_dir = light_ptr->generate();
		auto light_ray = ray(rec.p, light_dir);
		auto pdf_light = light_ptr->value(light_ray.direction());

		auto pdf_mat = rec.mat_ptr->pdf(r.direction(), rec, srec);
		

		if (coin < 0.5) {
			auto light_irr = rec.mat_ptr->eval(r.direction(), rec, light_dir);
			auto weight = pdf_light / (pdf_light + pdf_mat);

			return emitted + weight * light_irr * ray_color(light_ray, background, world, lights, depth - 1, mode,
				num_samples) / pdf_light;
		}
		else {
			auto mat_ray = ray(rec.p, srec.scattered);
			auto mat_irr = rec.mat_ptr->eval(r.direction(), rec, srec.scattered);
			auto weight = pdf_mat / (pdf_light + pdf_mat);
			return emitted + weight * mat_irr * ray_color(mat_ray, background, world, lights, depth - 1, mode,
				num_samples) / pdf_mat;
		}

	}
	return color(0, 0, 0);
}

int main(int argc, char *argv[]) {
	const int samples_per_pixel = atoi(argv[1]);
	const int pool_size = atoi(argv[2]);
	//const int regime = atoi(argv[2]);
	
	//const color background(0.0, 0.0, 0.0);

	const auto asp_ratio = 16.0 / 9.0;
	const int img_width = 500;
	const int img_height = static_cast<int>(img_width / asp_ratio);
	//const int max_depth = 10;

	const point3 loc = point3(50, 681.6-0.27, 81.6);
	const double radius = 600;
	const color col = color(15, 15, 15);

	hittable_list world = simple_scene(loc, radius, col);
	shared_ptr<hittable> lights = make_shared<sphere>(loc, radius, shared_ptr<material>());

	//add camera
	const vec3 vup = vec3(0, 1, 0);
	point3 lookfrom = point3(50, 50, 295.6);
	point3 lookat = point3(50, 50, 50);
	auto vfov = 30;
	camera cam(lookfrom, lookat, vup, vfov, asp_ratio);

	Renderer render(samples_per_pixel, asp_ratio, img_width);
	//AmbientOcclusionIntegrator simple;
	//RISMaterial ris(pool_size, 1);
	//PathTracerMaterials mats;
	//SimplePT pt;
	RISSimplePT rispt(pool_size, 1);
	BaseRNG sampler(samples_per_pixel);
	render.render_image(cam, rispt, world, sampler, lights);
	return 0;
}