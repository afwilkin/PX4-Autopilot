/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/**
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_h.h"
#include "ekf_derivation/generated/compute_hagl_innov_var.h"

void Ekf::controlRangeHaglFusion(const imuSample &imu_sample)
{
	_fc.rng.available = (_params.ekf2_rng_ctrl != static_cast<int32_t>(RngCtrl::DISABLED));

	static constexpr const char *HGT_SRC_NAME = "RNG";

	// Integrate only predicted vertical motion between range samples. Do not include
	// position corrections from range fusion in the step detector's reference.
	if (rangeStepEnabled()) {
		const float delta_range = -_state.vel(2) * imu_sample.delta_vel_dt;
		_rng_step_prediction += delta_range;
		_rng_step_candidate += delta_range;
		_rng_step_candidate_mean += delta_range;
		_rng_step_previous_surface += delta_range;
		_rng_step_current_surface += delta_range;

		for (auto &sample : _rng_step_history) {
			sample.prediction += delta_range;
		}
	}

	if (!rangeStepEnabled() || isTimedOut(_rng_step_last_sample, 300000)) {
		_rng_step_last_sample = 0;
		_rng_step_start = 0;
	}

	if (_rng_step_start != 0 && isTimedOut(_rng_step_start, 500000)) {
		// Bound the confirmation window even when no new range sample arrives.
		_rng_step_start = 0;
		_rng_step_last_sample = 0;
		_rng_step_cooldown = _time_delayed_us + 1000000;
	}

	bool rng_data_ready = false;

	if (_range_buffer) {
		// Get range data from buffer and check validity
		rng_data_ready = _range_buffer->pop_first_older_than(imu_sample.time_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.ekf2_rng_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.ekf2_rng_qlty_t);
		_range_sensor.setMaxFogDistance(_params.ekf2_rng_fog);

		_range_sensor.runChecks(imu_sample.time_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			if (_control_status.flags.in_air) {
				const bool horizontal_motion = _control_status.flags.fixed_wing
							       || (sq(_state.vel(0)) + sq(_state.vel(1)) > fmaxf(P.trace<2>(State::vel.idx), 0.1f));

				const float dist_dependant_var = sq(_params.ekf2_rng_sfe * _range_sensor.getDistBottom());
				const float var = sq(_params.ekf2_rng_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.ekf2_rng_k_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2),
							      P(State::vel.idx + 2, State::vel.idx + 2), horizontal_motion, imu_sample.time_us);
			}

		} else {
			// If we are supposed to be using range finder data but have bad range measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air
			    && _control_status.flags.vehicle_at_rest
			    && _range_sensor.isRegularlySendingData()
			    && _range_sensor.isDataReady()) {

				_range_sensor.setRange(_params.ekf2_min_rng);
				_range_sensor.setValidity(true); // bypass the checks
			}
		}

		_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	} else {
		return;
	}

	auto &aid_src = _aid_src_rng_hgt;

	if (rng_data_ready && _range_sensor.getSampleAddress()) {

		const float measurement = math::max(_range_sensor.getDistBottom(), _params.ekf2_min_rng);
		const float measurement_variance = getRngVar();

		float innovation_variance;
		sym::ComputeHaglInnovVar(P, measurement_variance, &innovation_variance);

		const float innov_gate = math::max(_params.ekf2_rng_gate, 1.f);
		updateAidSourceStatus(aid_src,
				      _range_sensor.getSampleAddress()->time_us, // sample timestamp
				      measurement,                               // observation
				      measurement_variance,                      // observation variance
				      getHagl() - measurement,                   // innovation
				      innovation_variance,                       // innovation variance
				      innov_gate);                               // innovation gate

		// Handle steps before height fusion, startup and timeout/reset paths can
		// interpret the innovation as vehicle motion.
		if (updateRangeStep(aid_src)) {
			return;
		}

		const bool measurement_valid = PX4_ISFINITE(aid_src.observation) && PX4_ISFINITE(aid_src.observation_variance);

		// z special case if there is bad vertical acceleration data, then don't reject measurement,
		// but limit innovation to prevent spikes that could destabilise the filter
		if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected
		    && measurement_valid && _range_sensor.isDataHealthy()
		   ) {
			const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
			aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
			aid_src.innovation_rejected = false;
		}

		const bool continuing_conditions_passing = _fc.rng.intended()
				&& _control_status.flags.tilt_align
				&& measurement_valid;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)
				&& _range_sensor.isRegularlySendingData()
				&& _range_sensor.isDataHealthy();

		const bool do_conditional_range_aid = (_control_status.flags.rng_terrain || _control_status.flags.rng_hgt)
						      && (_params.ekf2_rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL))
						      && isConditionalRangeAidSuitable();

		const bool do_range_aid = (_control_status.flags.rng_terrain || _control_status.flags.rng_hgt
					   || (rangeStepEnabled() && _rng_step_initialized))
					  && (_params.ekf2_rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED));

		if (_control_status.flags.rng_hgt) {
			if (!(do_conditional_range_aid || do_range_aid)) {
				ECL_INFO("stopping %s fusion", HGT_SRC_NAME);
				stopRngHgtFusion();
			}

		} else if (starting_conditions_passing) {
			if (_params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::RANGE)) {
				if (do_conditional_range_aid) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting conditional %s height fusion", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					if (!_control_status.flags.opt_flow_terrain && aid_src.innovation_rejected) {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}

				} else if (do_range_aid) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_information_events.flags.reset_hgt_to_rng = true;

					if (rangeStepEnabled() && _rng_step_initialized) {
						resetRangeHeight(aid_src);

					} else {
						resetAltitudeTo(aid_src.observation, aid_src.observation_variance);
						_state.terrain = 0.f;
						_rng_step_initialized = rangeStepEnabled();
					}

					resetAidSourceStatusZeroInnovation(aid_src);
					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					aid_src.time_last_fuse = imu_sample.time_us;
				}

			} else {
				if (do_conditional_range_aid || do_range_aid) {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					_control_status.flags.rng_hgt = true;

					if (!_control_status.flags.opt_flow_terrain && aid_src.innovation_rejected) {
						ECL_INFO("starting %s height fusion, resetting terrain", HGT_SRC_NAME);
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}
				}
			}
		}

		if (_control_status.flags.rng_hgt || _control_status.flags.rng_terrain) {
			if (continuing_conditions_passing) {

				if (do_conditional_range_aid) {
					_height_sensor_ref = HeightSensor::RANGE;

				} else if (_height_sensor_ref == HeightSensor::RANGE && !rangeStepEnabled()) {
					_height_sensor_ref = HeightSensor::UNKNOWN;
				}

				if (_range_sensor.isDataHealthy()
				    && _control_status.flags.rng_kin_consistent
				   ) {
					fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
				}

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()
				    && _control_status.flags.rng_hgt
				    && (_height_sensor_ref == HeightSensor::RANGE)
				    && starting_conditions_passing
				   ) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_rng = true;
					resetRangeHeight(aid_src);
					resetAidSourceStatusZeroInnovation(aid_src);

					// reset vertical velocity if no valid sources available
					if (!isVerticalVelocityAidingActive()) {
						resetVerticalVelocityToZero();
					}

					aid_src.time_last_fuse = imu_sample.time_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					if (_control_status.flags.opt_flow_terrain && isTerrainEstimateValid()) {
						ECL_WARN("stopping %s fusion, fusion failing", HGT_SRC_NAME);
						stopRngHgtFusion();
						stopRngTerrFusion();

					} else if (starting_conditions_passing) {
						if (rangeStepEnabled() && _rng_step_initialized) {
							resetRangeHeight(aid_src);

						} else {
							resetTerrainToRng(aid_src);
						}

						resetAidSourceStatusZeroInnovation(aid_src);
					}
				}

			} else {
				ECL_WARN("stopping %s fusion, continuing conditions failing", HGT_SRC_NAME);
				stopRngHgtFusion();
				stopRngTerrFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_control_status.flags.opt_flow_terrain) {
					if (!aid_src.innovation_rejected) {
						_control_status.flags.rng_terrain = true;
						fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
					}

				} else {
					if (aid_src.innovation_rejected) {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}

					_control_status.flags.rng_terrain = true;
				}
			}
		}

	} else if ((_control_status.flags.rng_hgt || _control_status.flags.rng_terrain)
		   && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s fusion, no data", HGT_SRC_NAME);
		stopRngHgtFusion();
		stopRngTerrFusion();
	}
}

float Ekf::getRngVar() const
{
	// In terrain-step mode, datum uncertainty is already represented by P and
	// the height/terrain correlation. Adding it to sensor noise again weakens
	// height correction progressively as terrain resets accumulate uncertainty.
	const float height_variance = rangeStepEnabled() ? 0.f : P(State::pos.idx + 2, State::pos.idx + 2);
	return fmaxf(
		       height_variance + sq(_params.ekf2_rng_noise)
		       + sq(_params.ekf2_rng_sfe * _range_sensor.getRange()),
		       0.f);
}

void Ekf::resetTerrainToRng(estimator_aid_source1d_s &aid_src)
{
	// Since the distance is not a direct observation of the terrain state but is based
	// on the height state, a reset should consider the height uncertainty. This can be
	// done by manipulating the Kalman gain to inject all the innovation in the terrain state
	// and create the correct correlation with the terrain state with a covariance update.
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 0.f);

	const float old_terrain = _state.terrain;

	VectorState H;
	sym::ComputeHaglH(&H);

	VectorState K;
	K(State::terrain.idx) = 1.f; // innovation is forced into the terrain state to create a "reset"

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	// record the state change
	const float delta_terrain = _state.terrain - old_terrain;

	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_terrain;
	}

	_state_reset_status.reset_count.hagl++;

	aid_src.time_last_fuse = _time_delayed_us;
}

bool Ekf::isConditionalRangeAidSuitable()
{
	// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
	// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
	float range_hagl_max = _params.ekf2_rng_a_hmax;
	float max_vel_xy = _params.ekf2_rng_a_vmax;

	const float hagl_test_ratio = _aid_src_rng_hgt.test_ratio;

	bool is_hagl_stable = (hagl_test_ratio < 1.f);

	if (!_control_status.flags.rng_hgt) {
		range_hagl_max = 0.7f * _params.ekf2_rng_a_hmax;
		max_vel_xy = 0.7f * _params.ekf2_rng_a_vmax;
		is_hagl_stable = (hagl_test_ratio < 0.01f);
	}

	const bool is_in_range = (getHagl() < range_hagl_max);

	bool is_below_max_speed = true;

	if (isHorizontalAidingActive()) {
		is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
	}

	return is_in_range && is_hagl_stable && is_below_max_speed;
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.rng_hgt = false;
	}
}

void Ekf::stopRngTerrFusion()
{
	_control_status.flags.rng_terrain = false;
}

bool Ekf::rangeStepEnabled() const
{
	return _params.ekf2_rng_step > 0.f
	       && _params.ekf2_rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED)
	       && _params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::RANGE);
}

void Ekf::resetRangeHeight(const estimator_aid_source1d_s &aid_src)
{
	// A range correction moves the vehicle relative to the stored surface. Moving
	// both states would preserve the old HAGL error and discard the terrain datum.
	const bool preserve_terrain = rangeStepEnabled() && _rng_step_initialized;
	resetAltitudeTo(aid_src.observation - _state.terrain,
			preserve_terrain ? aid_src.observation_variance : NAN, !preserve_terrain);
}

bool Ekf::updateRangeStep(estimator_aid_source1d_s &aid_src)
{
	if (!rangeStepEnabled() || !_rng_step_initialized || !_control_status.flags.in_air
	    || !_control_status.flags.rng_hgt || !_fc.rng.intended() || !_range_sensor.isDataHealthy()
	    || _fault_status.flags.bad_acc_vertical || !PX4_ISFINITE(aid_src.observation)) {
		_rng_step_last_sample = 0;
		_rng_step_start = 0;
		return false;
	}

	const uint64_t now = aid_src.timestamp_sample;
	const float measurement = aid_src.observation;
	const bool consecutive = _rng_step_last_sample != 0 && now > _rng_step_last_sample
				 && now - _rng_step_last_sample <= 300000;
	_rng_step_last_sample = now;

	// Use sensor noise, not height/terrain covariance: a step is a change between
	// observations and does not become less observable as datum uncertainty grows.
	const float noise = sqrtf(sq(_params.ekf2_rng_noise) + sq(_params.ekf2_rng_sfe * measurement));
	auto step_threshold = [&](float prediction) {
		// The difference contains noise from both surface distances. Using the
		// current distance twice makes an exit harder to confirm than an entry.
		const float prediction_variance = sq(_params.ekf2_rng_noise) + sq(_params.ekf2_rng_sfe * prediction);
		return math::max(_params.ekf2_rng_step, 3.f * sqrtf(prediction_variance + sq(noise)));
	};
	const float tolerance = math::constrain(noise, 0.05f, 0.1f);
	// Start probation before a smeared edge has already pulled height/velocity.
	// Cap only the probation noise contribution; confirmation requires either
	// a large change or stable observations on both sides of a persistent step.
	const float start_threshold = math::max(0.5f * _params.ekf2_rng_step, 2.f * math::min(noise, 0.15f));

	if (!consecutive) {
		_rng_step_last_confirmed = 0;
		_rng_step_start = 0;
		_rng_step_history_next = 0;

		for (auto &sample : _rng_step_history) {
			sample = {};
		}

	} else if (_rng_step_start != 0) {
		const float threshold = step_threshold(_rng_step_prediction);
		_rng_step_gate_passed |= fabsf(measurement - _rng_step_prediction) > threshold;

		if (!_rng_step_gate_passed && !_rng_step_return_candidate && !_rng_step_stable_baseline
		    && now - _rng_step_start >= 100000) {
			// Ordinary noise must not repeatedly withhold height corrections for
			// the full settling window without evidence of a stable previous surface.
			_rng_step_start = 0;
			_rng_step_last_sample = 0;
			_rng_step_cooldown = now + 1000000;
			return false;
		}

		if (fabsf(measurement - _rng_step_prediction) < tolerance) {
			// A single outlier or a surface crossed too briefly to confirm.
			_rng_step_start = 0;

		} else {
			if (fabsf(measurement - _rng_step_candidate) <= tolerance) {
				_rng_step_count++;
				_rng_step_candidate_mean += (measurement - _rng_step_candidate_mean) / _rng_step_count;
				const bool sustained_step = _rng_step_stable_baseline
							    && now - _rng_step_candidate_start >= 250000 && _rng_step_count >= 5
							    && fabsf(_rng_step_candidate_mean - _rng_step_prediction) > _params.ekf2_rng_step
							    && fabsf(measurement - _rng_step_prediction) > _params.ekf2_rng_step;

				if (now - _rng_step_candidate_start >= 150000 && _rng_step_count >= 3
				    && (fabsf(measurement - _rng_step_prediction) > threshold || sustained_step)) {
					resetTerrainToRng(aid_src);
					_time_last_terrain_fuse = _time_delayed_us;
					resetAidSourceStatusZeroInnovation(aid_src);
					_rng_step_start = 0;
					// Retain both observed surfaces briefly. A tilted beam can see
					// an edge long enough to confirm it, then return more slowly.
					_rng_step_last_confirmed = now;
					_rng_step_previous_surface = _rng_step_prediction;
					_rng_step_current_surface = measurement;
					_rng_step_history_next = 0;

					for (auto &sample : _rng_step_history) {
						sample = {};
					}

					// The derivative discontinuity was terrain, not an obstruction.
					_rng_consistency_check = RangeFinderConsistencyCheck{};
					_control_status.flags.rng_kin_consistent = true;
					return false;
				}

			} else {
				// The edge can span several observations. Allow its endpoint to
				// settle, but keep the original 0.5 s deadline and surface datum.
				_rng_step_candidate = measurement;
				_rng_step_candidate_mean = measurement;
				_rng_step_candidate_start = now;
				_rng_step_count = 1;
			}

			return true;
		}

	} else if (now >= _rng_step_cooldown) {
		auto start_candidate = [&](float prediction, bool returning = false) {
			// A stable pre-edge window permits confirmation from persistence,
			// without treating the configured fusion uncertainty as white noise.
			float sum = 0.f;
			float minimum = INFINITY;
			float maximum = -INFINITY;
			unsigned count = 0;
			uint64_t oldest = now;
			uint64_t newest = 0;

			for (const auto &sample : _rng_step_history) {
				if (sample.time_us != 0 && now > sample.time_us
				    && now - sample.time_us >= 40000 && now - sample.time_us <= 300000) {
					sum += sample.prediction;
					minimum = math::min(minimum, sample.prediction);
					maximum = math::max(maximum, sample.prediction);
					oldest = math::min(oldest, sample.time_us);
					newest = math::max(newest, sample.time_us);
					count++;
				}
			}

			_rng_step_stable_baseline = count >= 3 && newest - oldest >= 100000
						    && maximum - minimum <= tolerance;

			if (_rng_step_stable_baseline) {
				prediction = sum / count;
			}

			if (returning) {
				// The recently confirmed surface already passed settling checks.
				prediction = _rng_step_current_surface;
				_rng_step_stable_baseline = true;
			}

			_rng_step_start = now;
			_rng_step_prediction = prediction;
			_rng_step_candidate = measurement;
			_rng_step_candidate_mean = measurement;
			_rng_step_candidate_start = now;
			_rng_step_count = 1;
			_rng_step_gate_passed = fabsf(measurement - prediction) > step_threshold(prediction);
			_rng_step_return_candidate = returning;
		};

		const float return_change = measurement - _rng_step_current_surface;
		const float previous_change = _rng_step_previous_surface - _rng_step_current_surface;

		if (_rng_step_last_confirmed != 0 && now - _rng_step_last_confirmed <= 1000000
		    && fabsf(return_change) > tolerance && return_change * previous_change > 0.f
		    && fabsf(measurement - _rng_step_previous_surface) < fabsf(previous_change)) {
			// Only a return toward a recently observed surface gets this early
			// probation. IMU motion propagates both surfaces; the confirmation
			// and settling checks still apply, with the same 0.5 s deadline.
			start_candidate(_rng_step_current_surface, true);
			return true;
		}

		// Compare against all recent samples, not just the immediately preceding
		// one. Filtering and the sensor footprint can spread a sharp edge across
		// multiple readings, especially when leaving a raised surface.
		for (const auto &sample : _rng_step_history) {
			if (sample.time_us != 0 && now > sample.time_us && now - sample.time_us <= 100000
			    && fabsf(measurement - sample.prediction) > start_threshold) {
				start_candidate(sample.prediction);
				return true;
			}
		}
	}

	_rng_step_history[_rng_step_history_next] = {now, measurement};
	_rng_step_history_next = (_rng_step_history_next + 1) % RNG_STEP_HISTORY_LENGTH;
	return false;
}
