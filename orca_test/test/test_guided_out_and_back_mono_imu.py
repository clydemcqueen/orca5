"""
Launch test for Orca5 simulation (mono_imu): dive to 1 m off seafloor, execute Guided out-and-back maneuver,
and verify return to start position, depth maintenance, and transition to POSHOLD.
"""

import launch_testing.markers
import pytest

from orca_test import GuidedOutAndBackScenarioMixin, build_sim_test_description, stage_out_and_back_lua


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    stage_out_and_back_lua(__file__)
    return build_sim_test_description(mono_imu=True, enable_slam=False)


class TestGuidedOutAndBackMonoImu(GuidedOutAndBackScenarioMixin):
    MONO_IMU = True
    HARNESS_NAME = 'test_guided_out_and_back_mono_imu_harness'

    def test_out_and_back_mono_imu(self):
        self.run_out_and_back()
