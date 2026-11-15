# Mono IMU Notes

## What is VIBA?

VIBA stands for Visual-Inertial Bundle Adjustment — ORB-SLAM3's 3-stage inertial initialization/refinement pipeline in LocalMapping::Run(). Because monocular visual-inertial observability (gravity direction, metric scale, velocity, gyro bias, and especially accelerometer bias) takes several seconds of motion to become well-conditioned, ORB-SLAM3 refines the map in three progressive stages based on mTinit (accumulated moving time):

| Stage | Trigger | What `InitializeIMU(priorG, priorA, bFIBA)` does | State flags unlocked |
| --- | --- | --- | --- |
| Initial IMU Init | `>= 10` KFs & `>= 2.0 s` | Runs inertial-only MAP (`priorG=1e2`, `priorA=1e10` — effectively freezing accel bias near zero and heavily regularizing gyro bias) to estimate gravity `Rwg`, scale `s`, and velocities, scales/rotates the map, then runs `FullInertialBA` (`bFIBA=true`). | `isImuInitialized() = true`, `imu_initialized` becomes `True` in `/slam_status`. |
| VIBA 1 | `mTinit > 5.0 s` | Re-runs `InitializeIMU(1.f, 1e5, true)` — relaxes the bias priors (`priorG=1.0`, `priorA=1e5`) and runs a second `FullInertialBA` over all keyframes and map points so scale, gravity, and biases converge across the `5 s` window. | `SetIniertialBA1()`. |
| VIBA 2 | `mTinit > 15.0 s` | Re-runs `InitializeIMU(1.f, 1e4f, true)` in your fork (`5ddc1d8`; upstream uses `0.f, 0.f`) — loosens the accel bias prior one more decade (`1e5 -> 1e4`) and runs a final `FullInertialBA`. | `SetIniertialBA2()`. |
| (Later) `ScaleRefinement` | `mTinit` at `25, 35, ..., 75 s` | Runs inertial-only scale/gravity polish (if `KeyFramesInMap <= 200`). | — |