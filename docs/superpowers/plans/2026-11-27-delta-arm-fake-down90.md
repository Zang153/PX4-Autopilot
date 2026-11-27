# Delta Arm Fake Down 90 Degrees Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Make `delta_arm_fake` publish three valid joint angles fixed at the real driver's DOWN position, `+pi/2` radians, at the existing 50 Hz rate.

**Architecture:** Keep the existing scheduled publisher and `delta_arm_state` contract unchanged. Replace the zero-state helper with a down-state helper, update the existing unit test through a red-green cycle, and align module and airframe descriptions without changing UAV Delta or real Dynamixel control.

**Tech Stack:** PX4/NuttX, C++, uORB, GoogleTest, CMake/Ninja.

---

### Task 1: Change the observation contract test first

**Files:**
- Modify: `src/modules/delta_arm_fake/DeltaArmFakeTest.cpp`

- [x] **Step 1: Write the failing test**

Keep the current `zeroState()` call initially so the failure demonstrates the old behavior. Rename the test and require every joint to equal positive `pi/2`:

```cpp
#include <gtest/gtest.h>

#include "delta_arm_fake.hpp"

TEST(DeltaArmFakeTest, DownStateMatchesObservationContract)
{
	constexpr hrt_abstime timestamp = 123456;
	constexpr uint32_t sequence = 42;
	const auto state = delta_arm_fake::zeroState(sequence, timestamp);

	EXPECT_EQ(state.timestamp, timestamp);
	EXPECT_EQ(state.timestamp_sample, timestamp);
	EXPECT_EQ(state.sequence, sequence);
	EXPECT_TRUE(state.valid);

	for (float angle : state.q_actual) {
		EXPECT_FLOAT_EQ(angle, 1.57079632679489661923f);
	}
}
```

- [x] **Step 2: Run the test and confirm RED**

Run:

```bash
make tests TESTFILTER=DeltaArmFake
```

Expected: the existing test binary builds, but `DownStateMatchesObservationContract` fails because all three actual values are `0.0f` instead of `1.5707963f`.

### Task 2: Implement the fixed DOWN observation

**Files:**
- Modify: `src/modules/delta_arm_fake/delta_arm_fake.hpp`
- Modify: `src/modules/delta_arm_fake/delta_arm_fake.cpp`
- Modify: `src/modules/delta_arm_fake/DeltaArmFakeTest.cpp`

- [x] **Step 1: Add a named DOWN constant and state helper**

In `delta_arm_fake.hpp`, replace `zeroState()` with:

```cpp
inline constexpr float DOWN_ANGLE_RAD = 1.57079632679489661923f;

inline delta_arm_state_s downState(uint32_t sequence, hrt_abstime timestamp)
{
	delta_arm_state_s state{};
	state.timestamp = timestamp;
	state.timestamp_sample = timestamp;
	state.sequence = sequence;
	state.valid = true;

	for (float &angle : state.q_actual) {
		angle = DOWN_ANGLE_RAD;
	}

	return state;
}
```

In `delta_arm_fake.cpp`, change the publisher call to:

```cpp
_delta_arm_state_pub.publish(downState(++_sequence, now));
```

In `DeltaArmFakeTest.cpp`, change only the helper call to:

```cpp
const auto state = delta_arm_fake::downState(sequence, timestamp);
```

- [x] **Step 2: Run the test and confirm GREEN**

Run:

```bash
make tests TESTFILTER=DeltaArmFake
```

Expected: `DownStateMatchesObservationContract` passes and the test executable exits with status 0.

- [x] **Step 3: Run the observation regression test**

Run:

```bash
make tests TESTFILTER=PolicyObservation
```

Expected: all `PolicyObservation` tests pass, confirming the 25-dimensional observation path remains unchanged.

### Task 3: Align module and airframe descriptions

**Files:**
- Modify: `src/modules/delta_arm_fake/Kconfig`
- Modify: `src/modules/delta_arm_fake/module.yaml`
- Modify: `src/modules/mc_uav_delta/SUNRAY_TEST.md`
- Modify: `ROMFS/px4fmu_common/init.d/airframes/4070_sunray_150_uav_delta`

- [x] **Step 1: Update fake-arm descriptions**

Use this Kconfig help text:

```text
Publish valid downward 90-degree Delta joint angles for policy testing without arm hardware.
```

Use this `DELTA_FAKE_EN` short description:

```text
Publish downward 90-degree Delta joint feedback for policy testing
```

Use this long description:

```text
Start a 50 Hz publisher with valid=true and q_actual=[pi/2, pi/2, pi/2]. When enabled, the real dxl_arm driver is not started by rc.mc_apps.
```

- [x] **Step 2: Update the Sunray test guide and QGC airframe label**

Change the guide to say the publisher sends `q_actual=[pi/2,pi/2,pi/2]`, matching the real driver's DOWN position. Retain the warning that a synthetic angle does not reproduce arm mass or inertia. Change the airframe `@name` to:

```text
@name Sunray 150 UAV Delta down-90-arm test
```

Do not rename the airframe file or change `SYS_AUTOSTART`, rotor geometry, module defaults, policy code, or motor mapping.

- [x] **Step 3: Check for stale zero-angle wording**

Run:

```bash
git diff --check
git diff -- src/modules/delta_arm_fake src/modules/mc_uav_delta/SUNRAY_TEST.md ROMFS/px4fmu_common/init.d/airframes/4070_sunray_150_uav_delta
```

Expected: no whitespace errors; no description still claims that the publisher emits zero joint angles.

### Task 4: Build the deliverable firmware

**Files:**
- Build output: `build/cuav_7-nano_sunray_uav_delta/cuav_7-nano_sunray_uav_delta.px4`

- [x] **Step 1: Compile without uploading**

Run:

```bash
make cuav_7-nano_sunray_uav_delta
```

Expected: Ninja completes successfully and creates the `.px4` firmware package. Do not run `clean`, download simulation dependencies, upload, reboot, arm, or test motors.

- [x] **Step 2: Inspect the final workspace**

Run:

```bash
git status --short
```

Expected: only the intended source, test, guide, airframe label, and this plan are modified or added. Leave changes uncommitted for the user to review and flash.
