#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "utils.h"

class Integrator {
public:
	virtual ~Integrator() {}

	virtual color Li(
		const ray& r
	) const {
		return color();
	}
};

#endif
