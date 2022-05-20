/*
	This file is part of darts – the Dartmouth Academic Ray Tracing Skeleton.
	Copyright (c) 2017-2021 by Wojciech Jarosz
*/

#ifndef SAMPLER_H
#define SAMPLER_H

#include "vec3.h"
#include "pcg32.h"
#include <memory>


using std::unique_ptr;

class Sampler {
public:
	Sampler() : num_samples(1), cur_sample(0), cur_dim(0) {}
	virtual ~Sampler() {}

	virtual unique_ptr<Sampler> clone() const = 0;

	virtual void seed(int x, int y) {
		cur_dim = 0;
		cur_sample = 0;
	}

	virtual void start_pixel(int x, int y) = 0;
	virtual void advance() {
		cur_dim = 0;
		cur_sample++;
	}

	virtual double sample1d() = 0;
	virtual vec2 sample2d() = 0;
	virtual vec3 sample3d() = 0;
	virtual int sample_count() const {
		return num_samples;
	}

	void set_base_seed(int base) { 
		base_seed = base; 
	}

	int current_sample() const {
		return cur_sample;
	}

	int current_dim() const {
		return cur_dim;
	}

public:
	int num_samples;
	int cur_sample;
	int cur_dim;
	int base_seed;
};

class BaseRNG : public Sampler {
public: 
	BaseRNG(const int& nsamples) { num_samples = nsamples; }
	virtual ~BaseRNG() {}

	unique_ptr<Sampler> clone() const override {
		unique_ptr<BaseRNG> cloned(new BaseRNG());
		cloned->num_samples = num_samples;
		cloned->base_seed = base_seed;
		cloned->cur_dim = cur_dim;
		cloned->cur_sample = cur_sample;
		cloned->rng = rng;

		return std::move(cloned);
	}

	void seed(int x, int y) override {
		Sampler::seed(x, y);
		rng.seed(base_seed + x, base_seed + y);
	}

	void start_pixel(int x, int y) override {}

	double sample1d() override {
		cur_dim++;
		return rng.nextDouble();
	}

	vec2 sample2d() override {
		cur_dim += 2;
		return vec2(rng.nextDouble(), rng.nextDouble());
	}

	vec3 sample3d() override {
		cur_dim += 3;
		return vec3(rng.nextDouble(), rng.nextDouble(), rng.nextDouble());
	}

public:
	BaseRNG() {}

	pcg32 rng;
};
#endif
