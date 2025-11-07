std_msgs/Header header

# Reasons that we did not send a VPD/VPE to ArduSub:
uint16 WAIT_TRANSFORMS = 1              # Set if we are waiting for static transforms to be available.
uint16 WAIT_SONAR_RF = 2                # Set if we are waiting for sonar_rf to be available.
uint16 WAIT_EKF = 4                     # Set if we are waiting for the EKF to enter const_pos or horiz_pos mode.
uint16 WAIT_SLAM = 8                    # Set if we are waiting for ORB_SLAM3 to start tracking.
# Problem encountered, but we still sent a VPD/VPE to ArduSub:
uint16 OK_OUTLIER = 16                  # Set if we are sending an outlier to ArduSub.
uint16 flags

float64 sonar_rf                        # Input to scale filter is sonar_rf / slam_rf
float64 slam_rf
float64 scale                           # Output from scale filter