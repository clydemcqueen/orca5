"""
Launch test for Orca5 simulation (mono_imu): dive to 1 m off seafloor in AUTO, switch to POSHOLD,
inject ocean currents, record station keeping in VPD mode, switch to VPE mode, and verify
tighter z station keeping under VPE.
"""

import launch_testing.markers
import pytest

from orca_test import VpeHoldScenarioMixin, build_sim_test_description


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return build_sim_test_description(mono_imu=True, enable_slam=True)


class TestVpeHoldMonoImu(VpeHoldScenarioMixin):
    MONO_IMU = True
    HARNESS_NAME = 'test_vpe_hold_mono_imu_harness'
    CALLER_FILE = __file__

    def test_vpe_station_keeping_mono_imu(self):
        self.run_vpe_station_keeping()
