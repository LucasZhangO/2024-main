# SWERVE Base
CHASSIS_DATA = {
    # These current limit parameters are per-motor in the swerve modules
    "drive_continuous_current_limit": 40,
    "azimuth_continuous_current_limit": 30,
    "drive_peak_current_limit": 60,
    "azimuth_peak_current_limit": 40,

    # Talon FX motor controllers can set peak_current_duration.
    # SparkMAX motor controllers can't.
    "drive_peak_current_duration": 0.01,
    "azimuth_peak_current_duration": 0.01,

    # time in seconds for propulsion motors to ramp up to full speed
    # reference: https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
    "open_loop_ramp_rate": 0.5,
    "closed_loop_ramp_rate": 0.5,

    # TODO: update these values   # Jim Done 
    "swerve_modules": { # steer_id, drive_id, encoder_id
        "RF": [10, 20, 21],
        "RB": [7, 6, 8],
        "LB": [4, 3, 5],
        "LF": [1, 0, 20]
    },
}