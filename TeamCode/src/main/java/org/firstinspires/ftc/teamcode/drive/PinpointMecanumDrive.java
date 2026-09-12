package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Pose2D;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Medium-level mecanum drivetrain using the goBILDA Pinpoint for localization.
 *
 * Coordinate convention used by the teleop:
 * +forward = robot moves forward
 * +right   = robot moves to the right
 * +CCW     = positive rotation
 */
public class PinpointMecanumDrive {

    public static final String FRONT_LEFT_NAME = "FL";
    public static final String FRONT_RIGHT_NAME = "FR";
    public static final String BACK_LEFT_NAME = "BL";
    public static final String BACK_RIGHT_NAME = "BR";
    public static final String PINPOINT_NAME = "pinpoint";

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private final GoBildaPinpointDriver pinpoint;

    // Tune these values from the real robot measurements.
    // X pod: sideways offset from the robot center, in millimeters.
    // Y pod: forward/backward offset from the robot center, in millimeters.
    private static final double PINPOINT_X_OFFSET_MM = 0.0;
    private static final double PINPOINT_Y_OFFSET_MM = 0.0;

    public PinpointMecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        configureMotors();
        configurePinpoint();
    }

    private void configureMotors() {
        // Standard mecanum setup. If a wheel spins the wrong way, change its
        // direction here rather than adding unexplained minus signs later.
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void configurePinpoint() {
        // These are intentionally placeholders until the physical pod offsets
        // are measured on the actual robot.
        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);

        // Change this to SWINGARM if those are the pods installed on the robot.
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // X pod should increase when moving forward.
        // Y pod should increase when moving left.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // At the beginning of a match the robot should be stationary while
        // the Pinpoint calibrates its IMU and establishes the starting pose.
        pinpoint.resetPosAndIMU();
    }

    /** Update the Pinpoint pose estimate. Call once every loop. */
    public void update() {
        pinpoint.update();
    }

    /**
     * Drive using field-oriented controls.
     *
     * @param forward field-relative forward command (-1 to 1)
     * @param strafe  field-relative right command (-1 to 1)
     * @param rotate  robot rotation command (-1 to 1)
     * @param speed   global speed multiplier (0 to 1)
     */
    public void driveFieldCentric(double forward, double strafe, double rotate, double speed) {
        double heading = getHeadingRadians();

        // Rotate the field-relative joystick vector into the robot coordinate system.
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double robotStrafe = strafe * cos - forward * sin;
        double robotForward = strafe * sin + forward * cos;

        // Mecanum inverse kinematics.
        double frontLeftPower = robotForward + robotStrafe + rotate;
        double backLeftPower = robotForward - robotStrafe + rotate;
        double frontRightPower = robotForward - robotStrafe - rotate;
        double backRightPower = robotForward + robotStrafe - rotate;

        double max = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(backLeftPower),
                                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        frontLeftPower /= max;
        backLeftPower /= max;
        frontRightPower /= max;
        backRightPower /= max;

        frontLeft.setPower(Range.clip(frontLeftPower * speed, -1.0, 1.0));
        backLeft.setPower(Range.clip(backLeftPower * speed, -1.0, 1.0));
        frontRight.setPower(Range.clip(frontRightPower * speed, -1.0, 1.0));
        backRight.setPower(Range.clip(backRightPower * speed, -1.0, 1.0));
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double getHeadingRadians() {
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public double getXmm() {
        return pinpoint.getPosition().getX(DistanceUnit.MM);
    }

    public double getYmm() {
        return pinpoint.getPosition().getY(DistanceUnit.MM);
    }

    /** Zero heading while keeping the current X/Y position. */
    public void zeroHeading() {
        pinpoint.setHeading(0.0);
    }
}
