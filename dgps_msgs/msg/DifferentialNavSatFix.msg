# Center/primary antenna navigation fix.
sensor_msgs/NavSatFix nmea

# Frame-corrected ENU yaw, normalized to [0, 2*pi), in radians.
float32 heading

# The same heading expressed in degrees, normalized to [0, 360).
float32 heading_deg

# Heading variance in rad^2.
float32 heading_covariance
