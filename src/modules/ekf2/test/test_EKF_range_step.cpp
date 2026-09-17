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
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

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

TEST_F(EkfRangeStepTest, UnconfirmedEdgeResumesFusion)
{
	const float terrain = ekf->getTerrainVertPos();
	sim._rng.setData(1.2f, 100);
	sim.runSeconds(0.1f);
	// Neither returns to the original floor nor confirms the candidate table.
	sim._rng.setData(1.5f, 100);
	sim.runSeconds(0.8f);
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
