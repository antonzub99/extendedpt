#include "hittable_list.h"
#include "sphere.h"
#include "camera.h"
#include "color.h"
#include "utils.h"
#include "material.h"

#include <iostream>

hittable_list simple_scene(const point3& loc, const double& radius, const color& col) {
	hittable_list objects;

	auto material_left = make_shared<lambertian>(color(0.75, 0.25, 0.25));
	auto material_right = make_shared<lambertian>(color(0.25, 0.25, 0.75));
	auto material_back = make_shared<lambertian>(color(0.75, 0.75, 0.75));
	//auto material_front = make_shared<lambertian>(color());
	auto material_bot = make_shared<lambertian>(color(0.75, 0.75, 0.75));
	auto material_top = make_shared<lambertian>(color(0.75, 0.75, 0.75));

	auto material_first = make_shared<dielectric>(1.5);
	auto material_second = make_shared<metal>(color(1, 1, 1), 0.2);

	auto difflight = make_shared<diffuse_light>(col);


	objects.add(make_shared<sphere>(point3(1e5+1, 40.8, 81.6), 1e5, material_left));
	objects.add(make_shared<sphere>(point3(-1e5 + 99, 40.8, 81.6), 1e5, material_right));
	objects.add(make_shared<sphere>(point3(50, 40.8, 1e5), 1e5, material_back));
	//objects.add(make_shared<sphere>(point3(50, 40.8, -1e5+170), 1e5, material_front));
	objects.add(make_shared<sphere>(point3(50, 1e5, 81.6), 1e5, material_bot));
	objects.add(make_shared<sphere>(point3(50, -1e5+81.6, 81.6), 1e5, material_top));
	objects.add(make_shared<sphere>(point3(27, 16.5, 47), 16.5, material_first));
	objects.add(make_shared<sphere>(point3(73, 16.5, 78), 16.5, material_second));
	objects.add(make_shared<sphere>(loc, radius, difflight));
	return objects;
}

color ray_color(
	const ray& r, 
	const color& background, 
	const hittable& world, 
	shared_ptr<hittable> lights,
	int depth
) {
	hit_record rec;

	if (depth <= 0)
		return color(0, 0, 0);

	if (!world.hit(r, 0.001, infty, rec))
		return background;

	scatter_record srec;
	color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);

	if (!rec.mat_ptr->scatter(r, rec, srec))
		return emitted;

	if (srec.is_specular) {
		return srec.attenuation * ray_color(srec.specular_ray, background, world, lights, depth - 1);
	}

	//auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);

	cosine_pdf p(rec.normal);

	//hittable_pdf light_pdf(lights, rec.p);
	//ray scattered = ray(rec.p, light_pdf.generate());
	//auto pdf_val = light_pdf.value(scattered.direction());

	ray scattered = ray(rec.p, p.generate());
	auto pdf_val = p.value(scattered.direction());

	return emitted + srec.attenuation * rec.mat_ptr->scattering_pdf(r, rec, scattered) * ray_color(scattered, background, world, lights, depth - 1) / pdf_val;
}

int main() {
	const auto asp_ratio = 16.0 / 9.0;
	const int img_width = 500;
	const int img_height = static_cast<int>(img_width / asp_ratio);
	const int samples_per_pixel = 20;
	const int max_depth = 10;
	color background(0.0, 0.0, 0.0);	

	const point3 loc = point3(50, 681.6-0.27, 81.6);
	const double radius = 600;
	const color col = color(15, 15, 15);

	hittable_list world = simple_scene(loc, radius, col);
	//auto difflight = make_shared<diffuse_light>(color(10, 10, 10));
	shared_ptr<hittable> lights = make_shared<sphere>(loc, radius, shared_ptr<material>());

	//add camera
	const vec3 vup = vec3(0, 1, 0);
	point3 lookfrom = point3(50, 50, 295.6);
	point3 lookat = point3(50, 50, 50);
	auto vfov = 30;
	camera cam(lookfrom, lookat, vup, vfov, asp_ratio);

	std::cout << "P3\n" << img_width << ' ' << img_height << "\n255\n";
	for (auto j = img_height - 1; j >= 0; --j) {
		std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
		for (auto i = 0; i < img_width; ++i) {
			color pixel_color(0, 0, 0);
			for (auto s = 0; s < samples_per_pixel; ++s) {
				auto u = (i + random_double()) / (img_width - 1);
				auto v = (j + random_double()) / (img_height - 1);
				ray r = cam.get_ray(u, v);
				pixel_color += ray_color(r, background, world, lights, max_depth);
			}
			write_color(std::cout, pixel_color, samples_per_pixel);
		}
	}
	std::cerr << "\nDone.\n";
}