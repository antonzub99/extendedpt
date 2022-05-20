#ifndef RENDERER_H
#define RENDERER_H

#include "integrator.h"
#include "sphere.h"
#include "camera.h"
#include "color.h"
#include <iostream>

class Renderer {
public:
	Renderer(
		const int n_samples, const double asp_ratio, const int width): 
		num_samples(n_samples), pic_width(width), pic_height(static_cast<int>(width / asp_ratio)), aspect_ratio(asp_ratio) {}
	
	virtual ~Renderer() {}
	virtual color render_pixel(
		int u,
		int v,
		int n_samples,
		camera& cam,
		Integrator& evaluator,
		const hittable& world,
		Sampler& sampler,
		shared_ptr<hittable>& lights
	) const {
		color output(0, 0, 0);

		recursion_record recursive_rays;

		recursive_rays.acc_color = color(1.0, 1.0, 1.0);
		recursive_rays.depth = 0;
		recursive_rays.max_depth = 100;
		recursive_rays.start_roulette = 5;
		recursive_rays.prob = 1.0;
		int max_depth = 5;

		for (auto sample = 0; sample < n_samples; ++sample) {
			auto new_u = (u + random_double()) / (pic_width - 1);
			auto new_v = (v + random_double()) / (pic_height - 1);
			ray r = cam.get_ray(new_u, new_v);
			output += evaluator.Li(r, world, sampler, lights, max_depth);
		}
		return output;
	}

	virtual void render_image(
		camera& cam, 
		Integrator& evaluator,
		const hittable& world,
		Sampler& sampler,
		shared_ptr<hittable>& lights) const {

		std::cout << "P3\n" << pic_width << ' ' << pic_height << "\n255\n";
		std::cerr << "\nWriting to image\n";
		for (auto j = pic_height - 1; j >= 0; --j) {
			std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
			for (auto i = 0; i < pic_width; ++i) {
				auto clr = Renderer::render_pixel(i, j, num_samples, cam, evaluator, world, sampler, lights);
				write_color(std::cout, clr, num_samples);
			}
		}
		std::cerr << "\nDone.\n";
	}

public:
	int num_samples;
	int pic_height;
	int pic_width;
	double aspect_ratio;
	
};



#endif
