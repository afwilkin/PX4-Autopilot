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

#include <gtest/gtest.h>
#include <random>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"
#include "range_step_log241.h"

class EkfRangeStepTest : public ::testing::Test
{
public:
	std::shared_ptr<Ekf> ekf{std::make_shared<Ekf>()};
	SensorSimulator sim{ekf};
	EkfWrapper wrapper{ekf};

	void SetUp() override
	{
		ekf->init(0);
		wrapper.disableBaroHeightFusion();
		wrapper.disableGpsFusion();
		wrapper.setRangeHeightRef();
		wrapper.enableRangeHeightFusion();
		wrapper.enableFlowFusion();
		ekf->getParamHandle()->ekf2_rng_step = 0.3f;
		ekf->getParamHandle()->ekf2_rng_sfe = 0.05f; // shipped range-dependent noise default
		sim.runSeconds(0.1f);
		ekf->set_in_air_status(false);
		ekf->set_vehicle_at_rest(true);
		sim._rng.setLimits(0.1f, 10.f);
		sim._rng.setData(2.f, 100);
		sim.startRangeFinder();
		sim._flow.setData(sim._flow.dataAtRest());
		sim.startFlow();
		ekf->set_optical_flow_limits(5.f, 0.1f, 10.f);
		sim.runSeconds(5.f);
		ekf->set_in_air_status(true);
		ekf->set_vehicle_at_rest(false);
		sim.runSeconds(5.f);
		ASSERT_TRUE(wrapper.isIntendingRangeHeightFusion());
		ASSERT_EQ(ekf->getHeightSensorRef(), HeightSensor::RANGE);
		ASSERT_NEAR(ekf->getHagl(), 2.f, 0.02f);
	}

	void prepareHighRange()
	{
		sim._rng.setRateHz(50);

		for (int i = 1; i <= 600; ++i) {
			sim._rng.setData(2.f + 4.54f * i / 600.f, 100);
			sim.runSeconds(0.02f);
		}

		// Let height/velocity and bias settle after the artificial range-only climb.
		sim.runSeconds(20.f);
	}

	void expectSurface(float distance, float altitude, float duration = 10.f)
	{
		sim._rng.setData(distance, 100);

		// Check the transient, not only the final convergence.
		for (int i = 0; i < int(duration * 10); ++i) {
			sim.runSeconds(0.1f);
			EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
			EXPECT_NEAR(ekf->getVelocity()(2), 0.f, 0.06f);
		}

		EXPECT_NEAR(ekf->getHagl(), distance, 0.03f);
		EXPECT_TRUE(wrapper.isIntendingRangeHeightFusion());
		EXPECT_TRUE(wrapper.isIntendingFlowFusion());
	}
};

TEST_F(EkfRangeStepTest, RaisedSurfaceStopAndReturn)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	ResetLoggingChecker resets(ekf);
	resets.capturePreResetState();
	expectSurface(1.2f, altitude, 30.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), -0.8f, 0.03f);
	expectSurface(2.f, altitude);
	resets.capturePostResetState();
	EXPECT_TRUE(resets.isVerticalPositionResetCounterIncreasedBy(0));
	EXPECT_TRUE(resets.isVerticalVelocityResetCounterIncreasedBy(0));
}

TEST_F(EkfRangeStepTest, DepartRaisedLaunchSurface)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(2.8f, altitude);
	EXPECT_NEAR(ekf->getTerrainVertPos(), 0.8f, 0.03f);
	expectSurface(2.f, altitude);
}

TEST_F(EkfRangeStepTest, VerticalMotionAfterStep)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 2.f);
	const float terrain = ekf->getTerrainVertPos();
	// Smooth actual descent and climb, with matching specific force and range.
	float distance = 1.2f;

	for (float direction : {1.f, -1.f}) {
		float velocity = 0.f;

		for (int i = 0; i < 200; ++i) {
			const float accel = direction * (i < 100 ? 0.5f : -0.5f);
			distance -= velocity * 0.01f + 0.5f * accel * 0.0001f;
			velocity += accel * 0.01f;
			sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + accel));
			sim._rng.setData(distance, 100);
			sim.runSeconds(0.01f);
		}

		sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
		sim.runSeconds(5.f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude + distance - 1.2f, 0.04f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
		EXPECT_NEAR(ekf->getVelocity()(2), 0.f, 0.04f);
	}
}

TEST_F(EkfRangeStepTest, SlowRangeCorrectionIsNotTerrain)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 2.f);
	const float terrain = ekf->getTerrainVertPos();

	// Even motion initially missed by the IMU must be corrected by normal range fusion.
	for (int i = 1; i <= 100; ++i) {
		sim._rng.setData(1.2f - 0.003f * i, 100);
		sim.runSeconds(0.1f);
	}

	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude - 0.3f, 0.06f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
}

TEST_F(EkfRangeStepTest, RangeLossAndReacquisition)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 2.f);
	const float terrain = ekf->getTerrainVertPos();
	sim.stopRangeFinder();
	sim.runSeconds(3.f);
	EXPECT_FALSE(wrapper.isIntendingRangeHeightFusion());
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	sim.startRangeFinder();
	sim.runSeconds(5.f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	expectSurface(2.f, altitude);
}

TEST_F(EkfRangeStepTest, ReacquisitionCorrectsHeightAgainstStoredSurface)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 2.f);
	const float terrain = ekf->getTerrainVertPos();
	sim.stopRangeFinder();
	sim.runSeconds(3.f);
	// No observation of the edge: cannot call this a terrain step. Retain the datum.
	sim._rng.setData(1.f, 100);
	sim.startRangeFinder();
	sim.runSeconds(5.f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude - 0.2f, 0.04f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_NEAR(ekf->getHagl(), 1.f, 0.03f);
}

TEST_F(EkfRangeStepTest, SingleOutlier)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	sim._rng.setData(1.2f, 100);
	sim.runSeconds(0.05f);
	expectSurface(2.f, altitude, 5.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), 0.f, 0.01f);
}

TEST_F(EkfRangeStepTest, DisabledRetainsRangeRelativeHeight)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.f;
	const float altitude = ekf->getLatLonAlt().altitude();
	sim._rng.setData(1.2f, 100);
	sim.runSeconds(15.f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude - 0.8f, 0.1f);
}

TEST_F(EkfRangeStepTest, FlowScaleTracksObservedSurface)
{
	// Establish horizontal motion using matching IMU and optical flow.
	for (int i = 1; i <= 100; ++i) {
		sim._imu.setAccelData(Vector3f(0.5f, 0.f, -CONSTANTS_ONE_G));
		auto flow = sim._flow.dataAtRest();
		flow.flow_rate(1) = -(i * 0.005f) / 2.f;
		sim._flow.setData(flow);
		sim.runSeconds(0.01f);
	}

	sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
	sim.runSeconds(5.f);
	const float altitude = ekf->getLatLonAlt().altitude();

	for (float distance : {1.2f, 2.f}) {
		auto flow = sim._flow.dataAtRest();
		flow.flow_rate(1) = -0.5f / distance;
		sim._flow.setData(flow);
		expectSurface(distance, altitude, 3.f);
		EXPECT_NEAR(ekf->getFlowVelBody()(0), 0.5f, 0.03f);
		EXPECT_NEAR(ekf->getVelocity()(0), 0.5f, 0.04f);
	}
}

TEST_F(EkfRangeStepTest, UnconfirmedEdgeEntersBoundedRecovery)
{
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setData(1.2f, 100);
	sim.runSeconds(0.1f);

	// Keep moving the endpoint so no surface settles for the confirmation time.
	// The bridge budget must expire into bounded recovery, not full fusion.
	for (int i = 0; i < 30; ++i) {
		sim._rng.setData(i % 2 ? 1.5f : 1.2f, 100);
		sim.runSeconds(0.05f);
	}

	EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 200000);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
}

TEST_F(EkfRangeStepTest, HeightTimeoutKeepsTerrainDatum)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 2.f);
	const float terrain = ekf->getTerrainVertPos();
	// An unclassified innovation large enough to fail normal height fusion.
	ekf->getParamHandle()->ekf2_rng_step = 5.f;
	ekf->getParamHandle()->ekf2_rng_gate = 1.f;
	ResetLoggingChecker resets(ekf);
	resets.capturePreResetState();
	sim._rng.setData(2.2f, 100);
	sim.runSeconds(12.f);
	resets.capturePostResetState();
	EXPECT_TRUE(resets.isVerticalPositionResetCounterIncreasedBy(1));
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude + 1.f, 0.05f);
	EXPECT_NEAR(ekf->getHagl(), 2.2f, 0.03f);
}

TEST_F(EkfRangeStepTest, BaroFallbackKeepsTerrainDatum)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 2.f);
	const float terrain = ekf->getTerrainVertPos();
	wrapper.enableBaroHeightFusion();
	sim.runSeconds(5.f);
	sim.stopRangeFinder();
	sim.runSeconds(3.f);
	EXPECT_EQ(ekf->getHeightSensorRef(), HeightSensor::BARO);
	EXPECT_FALSE(wrapper.isIntendingTerrainFlowFusion());
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.03f);
	sim.startRangeFinder();
	sim.runSeconds(5.f);
	EXPECT_EQ(ekf->getHeightSensorRef(), HeightSensor::RANGE);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.05f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.03f);
}

TEST_F(EkfRangeStepTest, LandingAndTakeoffKeepDatum)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 2.f);
	const float terrain = ekf->getTerrainVertPos();
	ekf->set_in_air_status(false);
	ekf->set_vehicle_at_rest(true);
	sim.runSeconds(2.f);
	ekf->set_in_air_status(true);
	ekf->set_vehicle_at_rest(false);
	sim.runSeconds(2.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.03f);
}

TEST_F(EkfRangeStepTest, LossDuringConfirmationDoesNotCommitCandidate)
{
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setData(1.2f, 100);
	sim.runSeconds(0.1f);
	sim.stopRangeFinder();
	sim.runSeconds(1.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	// No stale candidate should be confirmed on reacquisition.
	sim._rng.setData(2.f, 100);
	sim.startRangeFinder();
	sim.runSeconds(5.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_NEAR(ekf->getHagl(), 2.f, 0.03f);
	EXPECT_TRUE(wrapper.isIntendingRangeHeightFusion());
	EXPECT_TRUE(wrapper.isIntendingFlowFusion());
}

TEST_F(EkfRangeStepTest, NoisySurfaceRemainsPersistent)
{
	const float altitude = ekf->getLatLonAlt().altitude();

	for (int i = 0; i < 200; ++i) {
		const float noise = (i % 5 - 2) * 0.02f;
		sim._rng.setData(1.2f + noise, 100);
		sim.runSeconds(0.05f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.09f);
	}

	EXPECT_NEAR(ekf->getTerrainVertPos(), -0.8f, 0.06f);
	EXPECT_NEAR(ekf->getHagl(), 1.2f, 0.06f);
}

TEST_F(EkfRangeStepTest, LoggedTableExitsSpreadAcrossSamples)
{
	// Raw 50 Hz lidar samples from log_77_2026-9-16-20-16-34.ulg.
	// Each exit spans three samples; none exceeds the old one-sample gate.
	const float exits[2][5] = {
		{0.549958f, 0.695602f, 0.971847f, 1.309181f, 1.348779f},
		{0.693925f, 0.927393f, 1.235376f, 1.496319f, 1.496624f}
	};
	sim._rng.setRateHz(50);

	for (const auto &exit : exits) {
		// Lower the vehicle slowly to the logged flight height before crossing.
		const float initial_range = ekf->getHagl();
		const float floor_range = exit[0] + 0.8f;

		for (int i = 1; i <= 200; ++i) {
			sim._rng.setData(initial_range + (floor_range - initial_range) * i / 200.f, 100);
			sim.runSeconds(0.02f);
		}

		sim.runSeconds(5.f);
		const float altitude = ekf->getLatLonAlt().altitude();
		const float floor_terrain = ekf->getTerrainVertPos();
		expectSurface(exit[0], altitude, 3.f);
		ResetLoggingChecker resets(ekf);
		resets.capturePreResetState();

		for (float range : exit) {
			sim._rng.setData(range, 100);
			sim.runSeconds(0.02f);
		}

		expectSurface(exit[4], altitude, 10.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), floor_terrain, 0.06f);
		resets.capturePostResetState();
		EXPECT_TRUE(resets.isVerticalPositionResetCounterIncreasedBy(0));
		EXPECT_TRUE(resets.isVerticalVelocityResetCounterIncreasedBy(0));
	}
}

TEST_F(EkfRangeStepTest, SettledEndpointMustStillExceedStepThreshold)
{
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setData(1.2f, 100);
	sim.runSeconds(0.1f);
	// A transient started confirmation, but the remaining change is too small
	// to classify as terrain. Normal height correction must resume instead.
	sim._rng.setData(1.8f, 100);
	sim.runSeconds(2.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 200000);
}

TEST_F(EkfRangeStepTest, TemporaryEdgeReturnFromLog80)
{
	// Tilt-corrected observations near 239.5--240.0 s in log 80. A brief
	// table-edge return settles, then recovers more slowly than the 100 ms gate.
	// Translate the waveform to the fixture's range; keep IMU motion at zero to
	// isolate the surface change from the subsequent closed-loop descent.
	const float observations[] = {
		1.678f, 1.680f, 1.681f, 1.683f, 1.605f, 1.444f, 1.313f,
		1.210f, 1.128f, 1.066f, 1.019f, 0.987f, 0.965f, 0.953f,
		0.948f, 0.952f, 0.961f, 0.978f, 1.003f, 1.035f, 1.076f,
		1.126f, 1.187f, 1.258f, 1.342f, 1.438f, 1.551f, 1.679f,
		1.708f, 1.703f, 1.697f
	};
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setRateHz(50);

	for (float observation : observations) {
		sim._rng.setData(observation + 2.f - observations[0], 100);
		sim.runSeconds(0.02f);
	}

	expectSurface(2.f, altitude, 15.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.06f);
}

TEST_F(EkfRangeStepTest, RangeCorrectionAfterRepeatedCrossings)
{
	const float altitude = ekf->getLatLonAlt().altitude();

	for (int i = 0; i < 20; ++i) {
		expectSurface(1.2f, altitude, 1.f);
		expectSurface(2.f, altitude, 1.f);
	}

	const float terrain = ekf->getTerrainVertPos();

	// A gradual descent initially missed by the IMU must still be corrected
	// promptly by range. Growing datum uncertainty must not weaken this aid.
	for (int i = 1; i <= 100; ++i) {
		sim._rng.setData(2.f - 0.003f * i, 100);
		sim.runSeconds(0.05f);
	}

	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude - 0.3f, 0.06f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.03f);
}

TEST_F(EkfRangeStepTest, RealClimbTowardPreviousSurfaceImmediatelyAfterStep)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(1.2f, altitude, 0.5f);
	const float terrain = ekf->getTerrainVertPos();
	float distance = 1.2f;
	float velocity_up = 0.f;

	// While previous-surface memory is still active, really climb 0.8 m with
	// matching IMU motion. The range returns to 2 m without a terrain transition.
	for (int i = 0; i < 200; ++i) {
		const float accel_up = i < 100 ? 1.25f : -1.25f;
		distance += velocity_up * 0.008f + 0.5f * accel_up * sq(0.008f);
		velocity_up += accel_up * 0.008f;
		sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G - accel_up));
		sim._rng.setData(distance, 100);
		sim.runSeconds(0.008f);
	}

	sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
	sim.runSeconds(4.f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude + 0.8f, 0.04f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_NEAR(ekf->getVelocity()(2), 0.f, 0.04f);
}

TEST_F(EkfRangeStepTest, RangeNoiseDoesNotBecomeTerrain)
{
	std::mt19937 generator(80);
	std::normal_distribution<float> noise(0.f, sqrtf(sq(0.1f) + sq(0.05f * 2.f)));
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setRateHz(50);

	for (int i = 0; i < 1000; ++i) {
		sim._rng.setData(2.f + noise(generator), 100);
		sim.runSeconds(0.02f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.15f);
	}

	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 600000);
}

TEST_F(EkfRangeStepTest, PersistentStepBelowInstantaneousNoiseGate)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();
// A stable 0.35 m change exceeds the configured minimum, but not the
// instantaneous noise gate. Multiple settled readings can now confirm it.
	expectSurface(1.65f, altitude, 5.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain - 0.35f, 0.03f);
}

TEST_F(EkfRangeStepTest, PersistentChangeBelowMinimumResumesFusion)
{
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setData(1.8f, 100);
	sim.runSeconds(1.f);
	EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 200000);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
}

TEST_F(EkfRangeStepTest, HigherTableExitFromLog78)
{
	// Tilt-corrected 50 Hz range around 127.6 s in log 78. At this height,
	// using the longer return's noise for both surfaces prevents confirmation.
	const float observations[] = {
		2.351f, 2.350f, 2.347f, 2.346f, 2.342f, 2.539f,
		3.137f, 3.136f, 3.132f, 3.131f, 3.127f, 3.125f,
		3.121f, 3.119f, 3.116f, 3.114f, 3.112f, 3.109f,
		3.108f, 3.105f, 3.104f, 3.101f, 3.101f, 3.098f,
		3.099f, 3.096f, 3.098f, 3.095f
	};
	sim._rng.setRateHz(50);
	ekf->getParamHandle()->ekf2_rng_step = 0.3048f;

	// Reach the logged distance gradually without acquiring a terrain offset.
	for (int i = 1; i <= 200; ++i) {
		sim._rng.setData(2.f + (observations[0] - 2.f) * i / 200.f, 100);
		sim.runSeconds(0.02f);
	}

	sim.runSeconds(5.f);
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();
	ResetLoggingChecker resets(ekf);
	resets.capturePreResetState();

	// Isolate the measured edge from subsequent controller motion.
	for (float observation : observations) {
		sim._rng.setData(observation, 100);
		sim.runSeconds(0.02f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
	}

	expectSurface(3.095f, altitude, 10.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain + 0.8f, 0.08f);
	// Revisit the table and stop there, preserving the same vehicle altitude.
	expectSurface(observations[0], altitude, 10.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.08f);
	resets.capturePostResetState();
	EXPECT_TRUE(resets.isVerticalPositionResetCounterIncreasedBy(0));
	EXPECT_TRUE(resets.isVerticalVelocityResetCounterIncreasedBy(0));
}

TEST_F(EkfRangeStepTest, HigherRangeNoiseDoesNotBecomeTerrain)
{
	sim._rng.setRateHz(50);

	for (int i = 1; i <= 300; ++i) {
		sim._rng.setData(2.f + 1.1f * i / 300.f, 100);
		sim.runSeconds(0.02f);
	}

	sim.runSeconds(5.f);
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();
	std::mt19937 generator(78);
	std::normal_distribution<float> noise(0.f, sqrtf(sq(0.1f) + sq(0.05f * 3.1f)));

	for (int i = 0; i < 1000; ++i) {
		sim._rng.setData(3.1f + noise(generator), 100);
		sim.runSeconds(0.02f);
		// Allow for the larger sensor noise at 3.1 m (0.185 m standard deviation).
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.2f);
	}

	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 600000);
}

TEST_F(EkfRangeStepTest, HighAltitudeCrossingsFromLog81)
{
	prepareHighRange();
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();

// Log 81's first crossing: approximately 6.54 -> 5.74 m.
	for (int rate : {10, 50, 100}) {
		sim._rng.setRateHz(rate);
		// Populate the pre-edge window at the new rate before crossing.
		sim.runSeconds(1.f);
		expectSurface(5.74f, altitude, 5.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain - 0.8f, 0.06f);
		expectSurface(6.54f, altitude, 5.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.06f);
	}
}

TEST_F(EkfRangeStepTest, HighAltitudeIndependentAndCorrelatedNoise)
{
	prepareHighRange();
	const float terrain = ekf->getTerrainVertPos();
	const float altitude = ekf->getLatLonAlt().altitude();
	std::mt19937 generator(81);
	const float sigma = sqrtf(sq(0.1f) + sq(0.05f * 6.54f));
	std::normal_distribution<float> random(0.f, sigma);

	for (float correlation : {0.f, 0.9f}) {
		float noise = 0.f;

		for (int i = 0; i < 1500; ++i) {
			noise = correlation * noise + sqrtf(1.f - sq(correlation)) * random(generator);
			sim._rng.setData(6.54f + noise, 100);
			sim.runSeconds(0.02f);
			EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
			EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 1200000);
		}
	}
	sim._rng.setData(6.54f, 100);
	sim.runSeconds(20.f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
}

TEST_F(EkfRangeStepTest, HighAltitudeOutlierAndUnsettledEdge)
{
	prepareHighRange();
	const float terrain = ekf->getTerrainVertPos();
	const float altitude = ekf->getLatLonAlt().altitude();
	sim._rng.setData(5.74f, 100);
	sim.runSeconds(0.08f);
	expectSurface(6.54f, altitude, 3.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);

	for (int i = 0; i < 30; ++i) {
		sim._rng.setData(i % 2 ? 5.74f : 6.04f, 100);
		sim.runSeconds(0.05f);
	}

	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 200000);
}

TEST_F(EkfRangeStepTest, HighAltitudeRealVerticalMotionAfterStep)
{
	prepareHighRange();
	const float altitude = ekf->getLatLonAlt().altitude();
	expectSurface(5.74f, altitude, 3.f);
	const float terrain = ekf->getTerrainVertPos();
	float distance = 5.74f;

	for (float direction : {1.f, -1.f}) {
		float velocity_up = 0.f;

		for (int i = 0; i < 200; ++i) {
			const float accel_up = direction * (i < 100 ? 1.25f : -1.25f);
			distance += velocity_up * 0.008f + 0.5f * accel_up * sq(0.008f);
			velocity_up += accel_up * 0.008f;
			sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G - accel_up));
			sim._rng.setData(distance, 100);
			sim.runSeconds(0.008f);
		}

		sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
		sim.runSeconds(5.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.02f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude + distance - 5.74f, 0.06f);
	}
}

TEST_F(EkfRangeStepTest, HighAltitudeLossDuringPersistentConfirmation)
{
	prepareHighRange();
	const float terrain = ekf->getTerrainVertPos();
	const float altitude = ekf->getLatLonAlt().altitude();
	sim._rng.setData(5.74f, 100);
	sim.runSeconds(0.2f);
	sim.stopRangeFinder();
	sim.runSeconds(1.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	sim._rng.setData(6.54f, 100);
	sim.startRangeFinder();
	expectSurface(6.54f, altitude, 5.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
}

TEST_F(EkfRangeStepTest, BoxEntryAcrossShortSensorGaps)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();

	for (float gap : {0.12f, 0.16f, 0.20f}) {
		SCOPED_TRACE(gap);
		sim.runSeconds(1.f);
		sim.stopRangeFinder();
		sim.runSeconds(gap);
		sim._rng.setData(2.f - 0.508f, 100);
		sim.startRangeFinder();
		expectSurface(2.f - 0.508f, altitude, 2.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain - 0.508f, 0.04f);
		expectSurface(2.f, altitude, 2.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.04f);
	}
}

TEST_F(EkfRangeStepTest, AdjacentTwentyAndTwentyFiveInchBoxes)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();

	for (bool reverse : {false, true}) {
		SCOPED_TRACE(reverse);
		expectSurface(2.f - (reverse ? 0.635f : 0.508f), altitude, 2.f);
		expectSurface(2.f - (reverse ? 0.508f : 0.635f), altitude, 2.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain - (reverse ? 0.508f : 0.635f), 0.04f);
		expectSurface(2.f, altitude, 2.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.04f);
	}
}

TEST_F(EkfRangeStepTest, BriefAdjacentBoxesAcrossInputGap)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();

	for (bool reverse : {false, true}) {
		SCOPED_TRACE(reverse);
		sim.runSeconds(1.f);
		sim.stopRangeFinder();
		sim.runSeconds(0.16f);
		sim._rng.setData(2.f - (reverse ? 0.635f : 0.508f), 100);
		sim.startRangeFinder();
		// Neither box individually provides the required settling time.
		sim.runSeconds(0.1f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
		sim._rng.setData(2.f - (reverse ? 0.508f : 0.635f), 100);
		sim.runSeconds(0.1f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
		expectSurface(2.f, altitude, 3.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.04f);
	}
}

TEST_F(EkfRangeStepTest, AdjacentBoxEndpointsBeforeBaselineRefills)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();

	for (bool reverse : {false, true}) {
		sim.runSeconds(1.f);
		sim.stopRangeFinder();
		sim.runSeconds(0.16f);
		sim._rng.setData(2.f - (reverse ? 0.635f : 0.508f), 100);
		sim.startRangeFinder();
		sim.runSeconds(reverse ? 0.12f : 0.32f);
		sim._rng.setData(2.f - (reverse ? 0.508f : 0.635f), 100);
		sim.runSeconds(reverse ? 0.28f : 0.20f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
		expectSurface(2.f, altitude, 3.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.04f);
	}
}

TEST_F(EkfRangeStepTest, SmallStepImmediatelyAfterConfirmedBoxEntry)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setData(2.f - 0.508f, 100);
	sim.runSeconds(0.24f);
	expectSurface(2.f - 0.635f, altitude, 3.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain - 0.635f, 0.04f);
	expectSurface(2.f, altitude, 3.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.04f);
}

TEST_F(EkfRangeStepTest, SmallMinimumNoiseAndMovingSurface)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float terrain = ekf->getTerrainVertPos();
	std::mt19937 generator(236);
	std::normal_distribution<float> random(0.f, 0.1f);

	for (float correlation : {0.f, 0.9f}) {
		float noise = 0.f;

		for (int i = 0; i < 500; ++i) {
			noise = correlation * noise + sqrtf(1.f - sq(correlation)) * random(generator);
			sim._rng.setData(2.f + noise, 100);
			sim.runSeconds(0.04f);
			EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
			EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 1200000);
		}
	}

	sim._rng.setData(2.f, 100);
	sim.runSeconds(5.f);

	// A moving return never settles long enough to become a committed surface.
	for (int i = 0; i < 25; ++i) {
		sim._rng.setData(i % 2 ? 1.4f : 1.7f, 100);
		sim.runSeconds(0.04f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
		EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 1200000);
	}

	sim._rng.setData(2.f, 100);
	sim.runSeconds(2.f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	EXPECT_FALSE(ekf->range_step_status().pending);
}

TEST_F(EkfRangeStepTest, RealVerticalMotionAcrossShortInputGap)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float terrain = ekf->getTerrainVertPos();
	const float altitude = ekf->getLatLonAlt().altitude();
	float distance = 2.f;
	float velocity = 0.f;

	for (int i = 0; i < 200; ++i) {
		if (i == 80) { sim.stopRangeFinder(); }

		if (i == 96) { sim.startRangeFinder(); }

		const float accel = i < 100 ? 0.5f : -0.5f;
		distance -= velocity * 0.01f + 0.5f * accel * sq(0.01f);
		velocity += accel * 0.01f;
		sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + accel));
		sim._rng.setData(distance, 100);
		sim.runSeconds(0.01f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
	}

	sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
	sim.runSeconds(4.f);
	EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude + distance - 2.f, 0.04f);
	EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
}

TEST_F(EkfRangeStepTest, RangeStepDiagnosticsReportConfirmationAndInterruption)
{
	sim._rng.setRateHz(25);
	sim.runSeconds(1.f);
	const auto initial = ekf->range_step_status();
	sim._rng.setData(1.2f, 100);
	sim.runSeconds(0.16f);
	const auto pending = ekf->range_step_status();
	ASSERT_TRUE(pending.pending);
	EXPECT_EQ(pending.event, estimator_range_step_status_s::EVENT_STARTED);
	EXPECT_GT(pending.event_count, initial.event_count);
	EXPECT_NEAR(pending.sample_interval, 0.04f, 0.01f);
	EXPECT_GT(pending.confirmation_threshold, 0.f);

	sim.runSeconds(0.5f);
	const auto confirmed = ekf->range_step_status();
	EXPECT_FALSE(confirmed.pending);
	EXPECT_EQ(confirmed.event, estimator_range_step_status_s::EVENT_CONFIRMED);
	EXPECT_GT(confirmed.event_count, pending.event_count);
	EXPECT_NEAR(confirmed.terrain_delta, -0.8f, 0.04f);
	EXPECT_NEAR(confirmed.terrain, ekf->getTerrainVertPos(), 0.001f);

	sim._rng.setData(2.f, 100);
	sim.runSeconds(0.12f);
	ASSERT_TRUE(ekf->range_step_status().pending);
	sim.stopRangeFinder();
	sim.runSeconds(0.6f);
	const auto interrupted = ekf->range_step_status();
	EXPECT_FALSE(interrupted.pending);
	EXPECT_EQ(interrupted.event, estimator_range_step_status_s::EVENT_INTERRUPTED);
	EXPECT_GT(interrupted.timestamp_sample, confirmed.timestamp_sample);
}


TEST_F(EkfRangeStepTest, Log241EntryAndChainedExit)
{
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	auto replay = [&](const auto & samples) {
		const float altitude = ekf->getLatLonAlt().altitude();
		const float terrain = ekf->getTerrainVertPos();
		const float offset = ekf->getHagl() - samples[0].range;
		const uint64_t start = sim.getTime();
		sim.stopRangeFinder();

		for (const auto &point : samples) {
			const uint64_t sample_time = start + point.time_us;

			if (sample_time > sim.getTime()) { sim.runMicroseconds(sample_time - sim.getTime()); }

			estimator::sensor::rangeSample sample {};
			sample.time_us = sim.getTime();
			sample.rng = point.range + offset;
			sample.quality = 100;
			ekf->setRangeData(sample);
			EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.06f);
		}

		const float endpoint = samples[sizeof(samples) / sizeof(samples[0]) - 1].range;
		sim._rng.setData(endpoint + offset, 100);
		sim.startRangeFinder();
		expectSurface(endpoint + offset, altitude, 3.f);
		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain + endpoint - samples[0].range, 0.06f);
		EXPECT_EQ(ekf->range_step_status().event, estimator_range_step_status_s::EVENT_CONFIRMED);
	};
	sim.runSeconds(1.f);
	replay(range_step_log241::entry);
	sim.runSeconds(1.f);
	replay(range_step_log241::exit);
}

TEST_F(EkfRangeStepTest, Log241ClosedLoopRepeatedCrossings)
{
// Simple vertical point-mass/controller model, not a full PX4 vehicle simulator.
// Unlike range-only playback, this checks real simulated height and control
// response while the range measurement includes the resulting aircraft motion.
	ekf->getParamHandle()->ekf2_rng_step = 0.09144f;
	sim._rng.setRateHz(25);
	const float target = ekf->getLatLonAlt().altitude();
	const float initial_terrain = ekf->getTerrainVertPos();
	float actual_height = 2.f;
	float velocity_up = 0.f;
	auto tick = [&](float surface) {
		const float velocity_command = math::constrain(target - float(ekf->getLatLonAlt().altitude()), -1.f, 1.f);
		const float acceleration_up = math::constrain(2.f * (velocity_command + ekf->getVelocity()(2)), -1.f, 1.f);
		actual_height += velocity_up * 0.01f + 0.5f * acceleration_up * sq(0.01f);
		velocity_up += acceleration_up * 0.01f;
		sim._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G - acceleration_up));
		sim._rng.setData(actual_height - surface, 100);
		sim.runSeconds(0.01f);
		EXPECT_NEAR(actual_height, 2.f, 0.10f);
		EXPECT_NEAR(velocity_command, 0.f, 0.10f);
	};
	auto crossing = [&](const auto & samples, float start_surface) {
		unsigned index = 0;
		const unsigned length = sizeof(samples) / sizeof(samples[0]);

		for (uint32_t us = 0; us <= samples[length - 1].time_us + 600000; us += 10000) {
			while (index + 1 < length && samples[index + 1].time_us <= us) { index++; }

			tick(start_surface + samples[0].range - samples[index].range);
		}

		return start_surface + samples[0].range - samples[length - 1].range;
	};

	for (int round = 0; round < 5; ++round) {
		SCOPED_TRACE(round);
		const float surface = crossing(range_step_log241::entry, 0.f);

		for (int i = 0; i < 100; ++i) { tick(surface); }

		// Use the recorded chained-exit shape, scaled to return to the original floor.
		const auto &exit = range_step_log241::exit;
		const float exit_rise = exit[sizeof(exit) / sizeof(exit[0]) - 1].range - exit[0].range;
		unsigned index = 0;

		for (uint32_t us = 0; us < 1500000; us += 10000) {
			while (index + 1 < sizeof(exit) / sizeof(exit[0]) && exit[index + 1].time_us <= us) { index++; }

			tick(surface * (1.f - (exit[index].range - exit[0].range) / exit_rise));
		}

		for (int i = 0; i < 200; ++i) { tick(0.f); }

		EXPECT_NEAR(ekf->getTerrainVertPos(), initial_terrain, 0.08f);
	}
}

TEST_F(EkfRangeStepTest, ProlongedAmbiguityLimitsCorrectionAndReanchors)
{
	const float altitude = ekf->getLatLonAlt().altitude();
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setRateHz(25);
	sim.runSeconds(1.f);
	bool recovered = false;

	for (int i = 0; i < 50; ++i) {
		sim._rng.setData(i % 2 ? 1.2f : 1.5f, 100);
		sim.runSeconds(0.04f);
		const auto status = ekf->range_step_status();

		if (status.degraded) {
			recovered = true;
			EXPECT_LE(fabsf(ekf->aid_src_rng_hgt().innovation), 0.05001f);
			EXPECT_GE(ekf->aid_src_rng_hgt().observation_variance, 0.25f);
		}

		EXPECT_NEAR(ekf->getTerrainVertPos(), terrain, 0.01f);
		EXPECT_NEAR(ekf->getLatLonAlt().altitude(), altitude, 0.15f);
	}

	EXPECT_TRUE(recovered);
	EXPECT_TRUE(ekf->range_step_status().degraded);
	EXPECT_GT(ekf->aid_src_rng_hgt().time_last_fuse, sim.getTime() - 200000);
	ResetLoggingChecker resets(ekf);
	resets.capturePreResetState();
	sim._rng.setData(1.4f, 100);
	sim.runSeconds(0.6f);
	EXPECT_FALSE(ekf->range_step_status().degraded);
	EXPECT_EQ(ekf->range_step_status().event, estimator_range_step_status_s::EVENT_REANCHORED);
	EXPECT_NEAR(ekf->getHagl(), 1.4f, 0.04f);
	resets.capturePostResetState();
	EXPECT_TRUE(resets.isVerticalPositionResetCounterIncreasedBy(0));
	EXPECT_TRUE(resets.isVerticalVelocityResetCounterIncreasedBy(0));
}

TEST(RangeSurfaceTrackerTest, MotionUncertaintyExhaustsBridgeBeforeTimeLimit)
{
	RangeSurfaceTracker tracker;
	uint64_t now = 1000000;

	for (int i = 0; i < 20; ++i) {
		now += 40000;
		tracker.predict(now, 0.f, 0.01f);
		tracker.update(now, 2.f, 0.09144f, 0.1f, 0.05f, 0.01f, 0.f);
	}

	now += 40000;
	tracker.predict(now, 0.f, 0.01f);
	EXPECT_EQ(tracker.update(now, 1.4f, 0.09144f, 0.1f, 0.05f, 0.01f, 0.6f), RangeSurfaceTracker::Action::Hold);
	tracker.predict(now + 80000, 0.f, 4.f);
	EXPECT_TRUE(tracker.status().degraded);
	EXPECT_LT(tracker.status().transition_age, 0.2f);
	EXPECT_GE(tracker.status().motion_uncertainty, 0.15f);
	EXPECT_EQ(tracker.update(now + 80000, 1.4f, 0.09144f, 0.1f, 0.05f, 4.f, 0.6f), RangeSurfaceTracker::Action::Recover);
}

TEST(RangeSurfaceTrackerTest, LostTransitionCannotResumeFullFusionOnFirstSample)
{
	RangeSurfaceTracker tracker;
	uint64_t now = 1000000;

	for (int i = 0; i < 20; ++i) {
		now += 40000;
		tracker.predict(now, 0.f, 0.01f);
		tracker.update(now, 2.f, 0.09144f, 0.1f, 0.05f, 0.01f, 0.f);
	}

	now += 40000;
	tracker.predict(now, 0.f, 0.01f);
	EXPECT_EQ(tracker.update(now, 1.4f, 0.09144f, 0.1f, 0.05f, 0.01f, 0.6f), RangeSurfaceTracker::Action::Hold);
	now += 400000;
	tracker.predict(now, 0.f, 0.01f);
	EXPECT_TRUE(tracker.status().degraded);
	EXPECT_EQ(tracker.update(now, 1.4f, 0.09144f, 0.1f, 0.05f, 0.01f, 0.6f), RangeSurfaceTracker::Action::Recover);
	EXPECT_TRUE(tracker.blocksFlow());
}
