#pragma once

#include"util.h"

struct Frame final {
	int index = 0;
	double timeIntervalInSeconds = 1.0 / 60.0;

	double timeInSeconds() const {
		return index * timeIntervalInSeconds;
	}

	void advance() {
		++index;
	}

	void advance(unsigned int delta) {
		index += delta;
	}
};

class Animation {
public:

	void update(const Frame& frame);

protected:
	virtual void onUpdate(const Frame & frame) = 0;
	
};

void Animation::update(const Frame& frame) {
	onUpdate(frame);
}