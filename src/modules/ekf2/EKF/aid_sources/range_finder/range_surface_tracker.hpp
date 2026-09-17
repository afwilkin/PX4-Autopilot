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

#pragma once

#include <mathlib/mathlib.h>
#include <uORB/topics/estimator_range_step_status.h>

// Classifies surface changes without modifying EKF state or covariance.
// The caller owns fusion, terrain resets and degraded measurement weighting.
class RangeSurfaceTracker
{
public:
	enum class State : uint8_t { Tracking, Transition, Unresolved, Reacquiring };
	enum class Action { Fuse, Hold, Confirm, Recover, Reanchor };

	void predict(uint64_t now, float delta_range, float velocity_variance);
	void interrupt();
	Action update(uint64_t now, float measurement, float minimum_step, float noise_floor,
		      float noise_scale, float velocity_variance, float height_innovation);
	bool blocksFlow() const { return _state != State::Tracking; }
	bool recovering() const { return _state == State::Unresolved || _state == State::Reacquiring; }
	estimator_range_step_status_s status() const;
	void setTerrainDelta(float delta) { _status.terrain_delta = delta; }

private:
	static constexpr float sq(float value) { return value * value; }
	static constexpr uint64_t MAX_SAMPLE_GAP = 300000;
	static constexpr uint64_t MAX_BRIDGE_TIME = 1000000;
	static constexpr float MAX_BRIDGE_ERROR = 0.15f; // propagated vertical-motion standard deviation, m
	void event(uint8_t value);
	void clearHistory();
	void startEndpoint(uint64_t now, float measurement);
	bool settleEndpoint(uint64_t now, float measurement, float tolerance);
	void acceptSurface(uint64_t now, float measurement);
	void unresolved();

	State _state{State::Tracking};
	estimator_range_step_status_s _status {};
	uint64_t _now{0};
	uint64_t _last_sample{0};
	uint64_t _transition_start{0};
	uint64_t _candidate_start{0};
	uint64_t _last_confirmed{0};
	float _previous_surface{0.f};
	float _current_surface{0.f};
	struct Sample { uint64_t time_us{0}; float prediction{0.f}; };
	static constexpr unsigned HISTORY_LENGTH = 32;
	Sample _history[HISTORY_LENGTH] {};
	unsigned _history_next{0};
	float _prediction{0.f};
	float _candidate{0.f};
	float _candidate_mean{0.f};
	float _motion_uncertainty{0.f};
	bool _stable_baseline{false};
	unsigned _count{0};
	bool _gate_passed{false};
	bool _return_candidate{false};
};
