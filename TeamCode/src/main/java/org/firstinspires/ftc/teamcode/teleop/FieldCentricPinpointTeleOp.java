package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.PinpointMecanumDrive;

/**
 * BioBuzz medium-level TeleOp.
 *
 * Controls:
 * Left stick  = field-relative translation
 * Right stick X = rotation
 * Left trigger  = precision / slow mode
 * Options       = zero field heading
 */
@TeleOp(name = "BioBuzz - Field Centric Pinpoint", group = "BioBuzz")
public class FieldCentricPinpointTeleOp extends LinearOpMode {

    private static final double DEADZONE = 0.05;
    private static final double SLOW_MODE_SPEED = 0.35;

    private PinpointMecanumDrive drive;
    private boolean previousOptions = false;

    @Override
    public void runOpMode() {
        drive = new PinpointMecanumDrive(hardwareMap);

        telemetry.addLine("BioBuzz Field-Centric TeleOp");
        telemetry.addLine("Pinpoint initialized.");
        telemetry.addLine("Keep the robot still during initialization.");
        telemetry.addLine("Options = zero heading");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();

            // FTC gamepad Y is negative when pushed forward, so invert it.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            forward = applyDeadzone(forward, DEADZONE);
            strafe = applyDeadzone(strafe, DEADZONE);
            rotate = applyDeadzone(rotate, DEADZONE);

            // Left trigger activates precision driving.
            double speed = gamepad1.left_trigger > 0.05
                    ? SLOW_MODE_SPEED
                    : 1.0;

            // Edge detection prevents repeatedly resetting the heading while
            // the button is held down.
            if (gamepad1.options && !previousOptions) {
                drive.zeroHeading();
            }
            previousOptions = gamepad1.options;

            drive.driveFieldCentric(forward, strafe, rotate, speed);

            telemetry.addData("Heading", "%.1f deg", drive.getHeadingDegrees());
            telemetry.addData("X", "%.1f mm", drive.getXmm());
            telemetry.addData("Y", "%.1f mm", drive.getYmm());
            telemetry.addData("Mode", speed < 1.0 ? "PRECISION" : "FULL SPEED");
            telemetry.addData("Heading reset", "OPTIONS");
            telemetry.update();
        }

        drive.stop();
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }

        // Rescale the remaining range so that the joystick still reaches 1.0.
        double sign = Math.signum(value);
        double magnitude = (Math.abs(value) - deadzone) / (1.0 - deadzone);
        return sign * magnitude;
    }
}
