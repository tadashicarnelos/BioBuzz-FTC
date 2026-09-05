package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Competition drivetrain/follower configuration.
 *
 * Values that depend on the physical robot should be replaced with the results
 * of Pedro Pathing's tuning OpModes before competition.
 */
public class FConstants {

    public static FollowerConstants followerConstants = new FollowerConstants();

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)

            // Hardware Map names. These must match the Robot Configuration exactly.
            .leftFrontMotorName("FL")
            .leftRearMotorName("BL")
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")

            // Initial FTC mecanum convention. Verify all four directions on the robot.
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            // Faster response while avoiding unnecessary repeated motor writes.
            .motorCachingThreshold(0.005)

            // Useful for competition driving: release the stick and the robot brakes.
            .useBrakeModeInTeleOp(true)

            // Helps keep behavior more consistent as battery voltage changes.
            .useVoltageCompensation(true)
            .nominalVoltage(12.0);

    /*
     * Do not guess the following values. Tune them on the real robot:
     *
     * followerConstants.mass = robot mass in kilograms
     * driveConstants.xVelocity = forward velocity
     * driveConstants.yVelocity = lateral velocity
     * follower PIDF coefficients
     * zero-power acceleration
     * predictive braking, if enabled
     * centripetal scaling
     */
}
