/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <aid_sources/range_finder/range_surface_tracker.hpp>

void RangeSurfaceTracker::event(uint8_t value)
{
	_status.event = value;
	_status.event_count++;
	_status.event_timestamp = _now;
	_status.timestamp_sample = _now;
}

void RangeSurfaceTracker::clearHistory()
{
	_history_next = 0;

	for (auto &sample : _history) { sample = {}; }
}

void RangeSurfaceTracker::interrupt()
{
	if (_state != State::Tracking) { event(estimator_range_step_status_s::EVENT_INTERRUPTED); }

	_state = State::Tracking;
	_last_sample = 0;
	_transition_start = 0;
	_last_confirmed = 0;
	_stable_baseline = false;
	clearHistory();
}

void RangeSurfaceTracker::unresolved()
{
	_state = State::Unresolved;
	_last_confirmed = 0;
	clearHistory();
	event(estimator_range_step_status_s::EVENT_TIMEOUT);
}

void RangeSurfaceTracker::predict(uint64_t now, float delta_range, float velocity_variance)
{
	const float dt = _now != 0 && now > _now ? (now - _now) * 1e-6f : 0.f;
	_now = now;
	_prediction += delta_range;
	_candidate += delta_range;
	_candidate_mean += delta_range;
	_previous_surface += delta_range;
	_current_surface += delta_range;

	for (auto &sample : _history) { sample.prediction += delta_range; }

	if (_state == State::Transition) {
		// Treat velocity error as correlated over the bridge, not independent samples.
		_motion_uncertainty += sqrtf(math::max(velocity_variance, 0.f)) * dt;

		if (now - _transition_start >= MAX_BRIDGE_TIME || _motion_uncertainty >= MAX_BRIDGE_ERROR) {
			unresolved();
		}
	}

	if (_last_sample != 0 && now > _last_sample && now - _last_sample > MAX_SAMPLE_GAP) {
		if (_state != State::Tracking) {
			// Preserve uncertainty across a lost transition. Reacquisition must not
			// silently restore full height correction against the old surface.
			_state = State::Unresolved;
			event(estimator_range_step_status_s::EVENT_INTERRUPTED);
		}

		_last_sample = 0;
		_last_confirmed = 0;
		clearHistory();
	}
}

void RangeSurfaceTracker::startEndpoint(uint64_t now, float measurement)
{
	_candidate = measurement;
	_candidate_mean = measurement;
	_candidate_start = now;
	_count = 1;
}

bool RangeSurfaceTracker::settleEndpoint(uint64_t now, float measurement, float tolerance)
{
	if (fabsf(measurement - _candidate) > tolerance) {
		startEndpoint(now, measurement);
		event(estimator_range_step_status_s::EVENT_ENDPOINT_CHANGED);
		return false;
	}

	_count++;
	_candidate_mean += (measurement - _candidate_mean) / _count;
	return true;
}

void RangeSurfaceTracker::acceptSurface(uint64_t now, float measurement)
{
	_state = State::Tracking;
	_transition_start = 0;
	_last_confirmed = now;
	_previous_surface = _prediction;
	_current_surface = measurement;
	clearHistory();
// Seed the new reference immediately for adjacent surfaces.
	_history[_history_next++] = {now, measurement};
}

estimator_range_step_status_s RangeSurfaceTracker::status() const
{
	auto result = _status;
	result.state = static_cast<uint8_t>(_state);
	result.pending = _state == State::Transition;
	result.degraded = recovering();
	result.stable_baseline = _stable_baseline;
	result.prediction = _prediction;
	result.candidate = _candidate;
	result.transition_age = _transition_start != 0 && _now >= _transition_start ? (_now - _transition_start) * 1e-6f : 0.f;
	result.motion_uncertainty = _motion_uncertainty;
	return result;
}

RangeSurfaceTracker::Action RangeSurfaceTracker::update(uint64_t now, float measurement, float minimum_step,
		float noise_floor, float noise_scale, float velocity_variance, float height_innovation)
{
	_status.timestamp_sample = _now;
	_status.observation = measurement;
	const uint64_t previous_sample = _last_sample;
	const bool consecutive = previous_sample != 0 && now > previous_sample && now - previous_sample <= MAX_SAMPLE_GAP;
	_status.sample_interval = previous_sample != 0 && now > previous_sample ? (now - previous_sample) * 1e-6f : 0.f;
	_last_sample = now;
	const float noise = sqrtf(sq(noise_floor) + sq(noise_scale * measurement));
	const float tolerance = math::constrain(math::min(noise, 0.5f * minimum_step), 0.02f, 0.1f);
	auto step_threshold = [&](float prediction) {
		return math::max(minimum_step, 3.f * sqrtf(sq(noise_floor) + sq(noise_scale * prediction) + sq(noise)));
	};
	const float start_threshold = math::max(0.5f * minimum_step, 2.f * math::min(noise, 0.15f));

	if (recovering()) {
		if (_state == State::Unresolved || !consecutive) {
			startEndpoint(now, measurement);
			_state = State::Reacquiring;
			event(estimator_range_step_status_s::EVENT_REACQUIRING);

		} else if (settleEndpoint(now, measurement, tolerance) && now - _candidate_start >= 250000 && _count >= 5) {
			// Reestablish a usable clearance datum without claiming the old transition
			// was observable. EKF height uncertainty accumulated during the bridge remains.
			const bool changed_surface = (_stable_baseline || _gate_passed)
						     && fabsf(measurement - _prediction) > minimum_step
						     && fabsf(_candidate_mean - _prediction) > minimum_step;

			if (changed_surface) {
				acceptSurface(now, measurement);

			} else if (fabsf(height_innovation) <= 0.05f) {
				_state = State::Tracking;
				_transition_start = 0;
				_last_confirmed = 0;
				clearHistory();

			} else {
				// Without credible terrain evidence, keep limiting height correction
				// until the EKF and the settled measurement agree. A timer cannot authorize it.
				return Action::Recover;
			}

			event(changed_surface ? estimator_range_step_status_s::EVENT_REANCHORED
			      : estimator_range_step_status_s::EVENT_RETURNED);
			return changed_surface ? Action::Reanchor : Action::Fuse;
		}

		return Action::Recover;
	}

	if (!consecutive) {
		_last_confirmed = 0;
		_state = State::Tracking;
		clearHistory();

	} else if (_state == State::Transition) {
		const float threshold = step_threshold(_prediction);
		_status.confirmation_threshold = threshold;
		_gate_passed |= fabsf(measurement - _prediction) > threshold;

		if (!_gate_passed && !_return_candidate && !_stable_baseline && now - _transition_start >= 100000) {
			// Weak evidence from a noisy baseline does not justify inertial-only flight.
			// Use the same bounded recovery policy as any other unresolved transition.
			unresolved();
			event(estimator_range_step_status_s::EVENT_UNSTABLE);
			return Action::Recover;
		}

		if (fabsf(measurement - _prediction) < tolerance) {
			_state = State::Tracking;
			_transition_start = 0;
			event(estimator_range_step_status_s::EVENT_RETURNED);

		} else {
			if (settleEndpoint(now, measurement, tolerance)) {
				const bool persistent = _stable_baseline && now - _candidate_start >= 250000 && _count >= 5
							&& fabsf(_candidate_mean - _prediction) > minimum_step && fabsf(measurement - _prediction) > minimum_step;

				if (now - _candidate_start >= 150000 && _count >= 3
				    && (fabsf(measurement - _prediction) > threshold || persistent)) {
					acceptSurface(now, measurement);
					event(estimator_range_step_status_s::EVENT_CONFIRMED);
					return Action::Confirm;
				}

				// A settled sub-minimum change is not a terrain step. Resume ordinary
				// correction only after this evidence, not because an arbitrary deadline expired.
				if (now - _candidate_start >= 250000 && _count >= 5
				    && fabsf(measurement - _prediction) <= minimum_step && fabsf(_candidate_mean - _prediction) <= minimum_step) {
					_state = State::Tracking;
					_transition_start = 0;
					clearHistory();
					event(estimator_range_step_status_s::EVENT_RETURNED);
					return Action::Fuse;
				}
			}

			return Action::Hold;
		}

	} else {
		// Anchor the baseline window before a short input gap. All stored distances
		// have already been propagated with IMU vertical motion. Do not widen the
		// normal edge window: that would classify slow slopes as abrupt steps.
		const uint64_t baseline_time = now - previous_sample > 100000 ? previous_sample : now;
		float sum = 0.f;
		float minimum = INFINITY;
		float maximum = -INFINITY;
		unsigned count = 0;
		uint64_t oldest = baseline_time;
		uint64_t newest = 0;

		for (const auto &sample : _history) {
			if (sample.time_us != 0 && baseline_time > sample.time_us
			    && baseline_time - sample.time_us >= 40000 && baseline_time - sample.time_us <= 300000) {
				sum += sample.prediction;
				minimum = math::min(minimum, sample.prediction);
				maximum = math::max(maximum, sample.prediction);
				oldest = math::min(oldest, sample.time_us);
				newest = math::max(newest, sample.time_us);
				count++;
			}
		}

		const bool stable_baseline = count >= 3 && newest - oldest >= 100000
					     && maximum - minimum <= tolerance;
		// Only a tightly clustered baseline earns a lower candidate-start gate.
		// Confirmation still requires the configured minimum and persistence.
		const bool quiet_baseline = stable_baseline
					    && maximum - minimum <= math::min(0.03f, 0.25f * minimum_step);
		const float candidate_threshold = quiet_baseline
						  ? math::max(0.5f * minimum_step, math::max(0.03f, 3.f * (maximum - minimum)))
						  : start_threshold;

		auto start_candidate = [&](float prediction, bool returning = false) {
			_stable_baseline = stable_baseline;

			if (_stable_baseline) {
				prediction = sum / count;
			}

			if (returning) {
				// The recently confirmed surface already passed settling checks.
				prediction = _current_surface;
				_stable_baseline = true;
			}

			event(estimator_range_step_status_s::EVENT_STARTED);
			_status.confirmation_threshold = step_threshold(prediction);
			_transition_start = now;
			_motion_uncertainty = 0.f;
			_state = State::Transition;
			_prediction = prediction;
			_candidate = measurement;
			_candidate_mean = measurement;
			_candidate_start = now;
			_count = 1;
			_gate_passed = fabsf(measurement - prediction) > step_threshold(prediction);
			_return_candidate = returning;
		};

		_status.start_threshold = candidate_threshold;

		const float return_change = measurement - _current_surface;
		const float previous_change = _previous_surface - _current_surface;

		if (_last_confirmed != 0 && now - _last_confirmed <= 1000000
		    && fabsf(return_change) > tolerance && return_change * previous_change > 0.f
		    && fabsf(measurement - _previous_surface) < fabsf(previous_change)) {
			// Only a return toward a recently observed surface gets this early
			// probation. IMU motion propagates both surfaces; the confirmation
			// and settling checks still apply within the transition uncertainty budget.
			_status.start_threshold = tolerance;
			start_candidate(_current_surface, true);
			return Action::Hold;
		}

		// Compare against all recent samples, not just the immediately preceding
		// one. Filtering and the sensor footprint can spread a sharp edge across
		// multiple readings, especially when leaving a raised surface.
		for (const auto &sample : _history) {
			// Across a gap, use only the last observation, with a stable baseline
			// and an extra gate for uncertainty in the propagated vertical motion.
			const bool gap_reference = stable_baseline && sample.time_us == previous_sample
						   && now - sample.time_us <= 300000;
			const float motion_margin = gap_reference && now - sample.time_us > 100000
						    ? 3.f * sqrtf(math::max(velocity_variance, 0.f))
						    * (now - sample.time_us) * 1e-6f : 0.f;

			if (sample.time_us != 0 && now > sample.time_us
			    && (now - sample.time_us <= 100000 || gap_reference)
			    && fabsf(measurement - sample.prediction) > candidate_threshold + motion_margin) {
				_status.start_threshold = candidate_threshold + motion_margin;
				start_candidate(sample.prediction);
				return Action::Hold;
			}
		}
	}

	_history[_history_next] = {now, measurement};
	_history_next = (_history_next + 1) % HISTORY_LENGTH;
	return Action::Fuse;
}
