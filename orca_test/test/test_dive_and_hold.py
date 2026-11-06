"""
Launch test for Orca5 simulation (mono): dive to 1 m off seafloor and hold position for 5 seconds.
"""

import launch_testing.markers
import pytest

from orca_test import DiveAndHoldScenarioMixin, build_sim_test_description


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return build_sim_test_description(mono_imu=False, enable_slam=True)


class TestDiveAndHold(DiveAndHoldScenarioMixin):
    MONO_IMU = False
    HARNESS_NAME = 'test_dive_and_hold_harness'
    CALLER_FILE = __file__

    def test_dive_and_hold(self):
        self.run_dive_and_hold()
