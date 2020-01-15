#pragma once

#include"animation.h"
#include"particle_system_data3.h"

class ParticleEmitter3 {
public:
	//!
	//! \brief Callback function type for update calls.
	//!
	//! This type of callback function will take the emitter pointer, current
	//! time, and time interval in seconds.
	//!
	typedef std::function<void(ParticleEmitter3*, double, double)>
		OnBeginUpdateCallback;

	//! Default constructor.
	ParticleEmitter3();

	//! Destructor.
	virtual ~ParticleEmitter3();

	//! Updates the emitter state from \p currentTimeInSeconds to the following
	//! time-step.
	void update(double currentTimeInSeconds, double timeIntervalInSeconds);

	//! Returns the target particle system to emit.
	const ParticleSystemData3Ptr& target() const;

	//! Sets the target particle system to emit.
	void setTarget(const ParticleSystemData3Ptr& particles);

	//! Returns true if the emitter is enabled.
	bool isEnabled() const;

	//! Sets true/false to enable/disable the emitter.
	void setIsEnabled(bool enabled);

	//!
	//! \brief      Sets the callback function to be called when
	//!             ParticleEmitter3::update function is invoked.
	//!
	//! The callback function takes current simulation time in seconds unit. Use
	//! this callback to track any motion or state changes related to this
	//! emitter.
	//!
	//! \param[in]  callback The callback function.
	//!
	void setOnBeginUpdateCallback(const OnBeginUpdateCallback& callback);

protected:
	//! Called when ParticleEmitter3::update is executed.
	virtual void onUpdate(
		double currentTimeInSeconds,
		double timeIntervalInSeconds) = 0;

private:
	bool _isEnabled = true;
	ParticleSystemData3Ptr _particles;
	OnBeginUpdateCallback _onBeginUpdateCallback;
};

//! Shared pointer for the ParticleEmitter3 type.
typedef std::shared_ptr<ParticleEmitter3> ParticleEmitter3Ptr;

ParticleEmitter3::ParticleEmitter3() {}

ParticleEmitter3::~ParticleEmitter3() {}

const ParticleSystemData3Ptr& ParticleEmitter3::target() const {
	return _particles;
}

void ParticleEmitter3::setTarget(const ParticleSystemData3Ptr& particles) {
	_particles = particles;
}

bool ParticleEmitter3::isEnabled() const { return _isEnabled; }

void ParticleEmitter3::setIsEnabled(bool enabled) { _isEnabled = enabled; }

void ParticleEmitter3::update(double currentTimeInSeconds,
	double timeIntervalInSeconds) {
	if (_onBeginUpdateCallback) {
		_onBeginUpdateCallback(this, currentTimeInSeconds,
			timeIntervalInSeconds);
	}

	onUpdate(currentTimeInSeconds, timeIntervalInSeconds);
}

void ParticleEmitter3::setOnBeginUpdateCallback(
	const OnBeginUpdateCallback& callback) {
	_onBeginUpdateCallback = callback;
}