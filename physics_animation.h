#pragma once

#include"animation.h"
#include<tbb/tbb.h>

constexpr double kEpsilonD = std::numeric_limits<double>::epsilon();

class PhysicsAnimation : public Animation {
public:
	PhysicsAnimation();
	~PhysicsAnimation();

	double getCurrentTime() const;
	double currentTimeInSeconds() const;
protected:
	virtual void onAdvanceTimeStep(double timeIntervalInSeconds) = 0;
	virtual void onInitialize() = 0;
	void onUpdate(const Frame& frame) override final;
	virtual unsigned int numberOfSubTimeSteps(
		double timeIntervalInSeconds) const;
private:
	Frame _currentFrame;
	double _currentTime = 0.0;

	void advanceTimeStep(double timeIntervalInSeconds);
	void initialize();
};

PhysicsAnimation::PhysicsAnimation() {
	_currentFrame.index = -1;
}
PhysicsAnimation::~PhysicsAnimation() {}

double PhysicsAnimation::getCurrentTime() const {
	return _currentTime;
}

double PhysicsAnimation::currentTimeInSeconds() const { return _currentTime; }

void PhysicsAnimation::initialize() {
	onInitialize();
}

void PhysicsAnimation::onUpdate(const Frame& frame) {

	if (frame.index > _currentFrame.index) {
		if (_currentFrame.index < 0) {
			initialize();
		}

		size_t numberOfFrames = frame.index - _currentFrame.index;

		for (size_t i = 0; i < numberOfFrames; ++i) {
			advanceTimeStep(frame.timeIntervalInSeconds);
		}

		_currentFrame = frame;
	}

}

unsigned int PhysicsAnimation::numberOfSubTimeSteps(double timeIntervalInSeconds) const {
	return 0.01;
}

void PhysicsAnimation::advanceTimeStep(double timeIntervalInSeconds) {
	_currentTime = _currentFrame.timeInSeconds();
	double remainingTime = timeIntervalInSeconds;
	while (remainingTime > kEpsilonD) {
		unsigned int numSteps = numberOfSubTimeSteps(remainingTime);
		double actualTimeInterval =
			remainingTime / static_cast<double>(numSteps);
		onAdvanceTimeStep(actualTimeInterval);

		remainingTime -= actualTimeInterval;
		_currentTime += actualTimeInterval;

	}
}


