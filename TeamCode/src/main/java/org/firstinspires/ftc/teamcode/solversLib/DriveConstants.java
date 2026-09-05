package org.firstinspires.ftc.teamcode.solversLib;

public final class DriveConstants {
    private DriveConstants() {}

    // Change these after measuring/tuning the real robot.
    public static final double SLOW_MODE_MULTIPLIER = 0.35;
    public static final double TURN_SLOW_MODE_MULTIPLIER = 0.45;
    public static final double JOYSTICK_DEADBAND = 0.05;

    // GoBILDA Pinpoint offsets are in millimeters in the Pinpoint driver.
    // X offset = sideways distance from robot center to the forward pod.
    // Y offset = forward/backward distance from robot center to the strafe pod.
    // Keep at zero until the actual pod geometry is measured/tuned.
    public static final double PINPOINT_X_OFFSET_MM = 0.0;
    public static final double PINPOINT_Y_OFFSET_MM = 0.0;
}
