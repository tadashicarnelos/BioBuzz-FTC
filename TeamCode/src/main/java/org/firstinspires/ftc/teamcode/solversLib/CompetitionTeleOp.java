package org.firstinspires.ftc.teamcode.solversLib;

import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Competition-oriented mecanum TeleOp using SolversLib + GoBILDA Pinpoint.
 *
 * SolversLib performs the mecanum wheel mixing and field-centric transform.
 * The GoBILDA Pinpoint supplies the robot heading from its two odometry pods + IMU.
 */
@TeleOp(name = "Solvers Competition TeleOp", group = "Competition")
public class CompetitionTeleOp extends OpMode {
    private MecanumDrive drive;
    private PinpointLocalizer pinpoint;

    private boolean slowMode = false;
    private boolean robotCentric = false;
    private boolean lastSlowButton = false;
    private boolean lastModeButton = false;

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;

    @Override
    public void init() {
        frontLeft = new Motor(hardwareMap, "FL");
        frontRight = new Motor(hardwareMap, "FR");
        backLeft = new Motor(hardwareMap, "BL");
        backRight = new Motor(hardwareMap, "BR");

        // SolversLib's MecanumDrive automatically inverts the right side.
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        pinpoint = new PinpointLocalizer(hardwareMap);

        telemetry.addLine("SolversLib TeleOp ready");
        telemetry.addLine("LB: precision mode");
        telemetry.addLine("B: Field-Centric / Robot-Centric");
        telemetry.update();
    }

    @Override
    public void start() {
        pinpoint.reset();
    }

    @Override
    public void loop() {
        pinpoint.update();
        updateToggles();

        double forward = shapeInput(-gamepad1.left_stick_y);
        double strafe = shapeInput(-gamepad1.left_stick_x);
        double turn = shapeInput(-gamepad1.right_stick_x);

        if (slowMode) {
            forward *= DriveConstants.SLOW_MODE_MULTIPLIER;
            strafe *= DriveConstants.SLOW_MODE_MULTIPLIER;
            turn *= DriveConstants.TURN_SLOW_MODE_MULTIPLIER;
        }

        double headingDegrees = pinpoint.getHeadingDegrees();

        if (robotCentric) {
            drive.driveRobotCentric(strafe, forward, turn);
        } else {
            drive.driveFieldCentric(strafe, forward, turn, headingDegrees);
        }

        updateTelemetry(headingDegrees);
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
    }

    private double shapeInput(double value) {
        value = applyDeadband(value, DriveConstants.JOYSTICK_DEADBAND);
        return value * value * value;
    }

    private double applyDeadband(double value, double deadband) {
        double magnitude = Math.abs(value);

        if (magnitude <= deadband) {
            return 0.0;
        }

        double scaled = (magnitude - deadband) / (1.0 - deadband);
        return Math.copySign(scaled, value);
    }

    private void updateToggles() {
        boolean slowButton = gamepad1.left_bumper;
        if (slowButton && !lastSlowButton) {
            slowMode = !slowMode;
        }
        lastSlowButton = slowButton;

        boolean modeButton = gamepad1.b;
        if (modeButton && !lastModeButton) {
            robotCentric = !robotCentric;
        }
        lastModeButton = modeButton;
    }

    private void updateTelemetry(double headingDegrees) {
        telemetry.addData("Drive Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
        telemetry.addData("Precision", slowMode ? "ON" : "OFF");
        telemetry.addData("Heading", "%.2f deg", headingDegrees);
        telemetry.addData("X", "%.2f in", pinpoint.getXInches());
        telemetry.addData("Y", "%.2f in", pinpoint.getYInches());
        telemetry.update();
    }
}
