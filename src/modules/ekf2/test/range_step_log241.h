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

// Corrected range observations from log_241_2026-9-17-16-18-22.ulg, EKF0.
// Times preserve logged EKF sample spacing. Excludes timeout-only status updates.
// Entry stops before the controller-induced climb; tests hold the final endpoint.
#pragma once
#include <cstdint>
namespace range_step_log241
{
struct Sample { uint32_t time_us; float range; };
static constexpr Sample entry[] = {
	{0, 1.127893f},
	{39669, 1.056560f},
	{79337, 0.994325f},
	{119006, 0.986308f},
	{158674, 0.944439f},
	{198343, 0.914043f},
	{238011, 0.896789f},
	{277680, 0.890629f},
	{317349, 0.870193f},
	{357017, 0.705274f},
	{396686, 0.671596f},
	{436354, 0.651039f},
	{476023, 0.640672f},
	{515691, 0.622944f},
};
static constexpr Sample exit[] = {
	{0, 0.668742f},
	{39669, 0.540891f},
	{158675, 0.537010f},
	{198344, 0.537008f},
	{247930, 0.544821f},
	{287598, 0.546784f},
	{327267, 0.959809f},
	{357019, 1.251737f},
	{406604, 1.254666f},
	{446273, 1.300581f},
};
}
