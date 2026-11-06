"""
Launch test for Orca5 simulation (mono): dive to 1 m off seafloor in AUTO, switch to POSHOLD,
inject ocean currents, record station keeping in VPD mode, switch to VPE mode, and verify
tighter z station keeping under VPE.
"""

import launch_testing.markers
import pytest

from orca_test import VpeHoldScenarioMixin, build_sim_test_description


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return build_sim_test_description(mono_imu=False, enable_slam=True)


class TestVpeHold(VpeHoldScenarioMixin):
    MONO_IMU = False
    HARNESS_NAME = 'test_vpe_hold_harness'
    CALLER_FILE = __file__

    def test_vpe_station_keeping(self):
        self.run_vpe_station_keeping()
