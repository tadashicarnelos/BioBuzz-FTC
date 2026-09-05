package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Competition drivetrain TeleOp.
 *
 * Features:
 * - Mecanum drive through Pedro Pathing
 * - Field-centric control using Pinpoint localization/IMU heading
 * - Toggleable robot-centric mode for testing/troubleshooting
 * - Toggleable precision/slow mode
 * - Joystick deadband + cubic input shaping for fine control
 * - Live X/Y/heading/velocity telemetry
 */
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompetitionTeleOp extends OpMode {

    private Follower follower;

    private boolean slowMode = false;
    private boolean robotCentric = false;

    private boolean lastSlowButton = false;
    private boolean lastModeButton = false;

    private static final double SLOW_MODE_MULTIPLIER = 0.35;
    private static final double TURN_SLOW_MODE_MULTIPLIER = 0.45;
    private static final double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Start at the field origin. Set a real starting pose in Auto instead.
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Competition TeleOp ready");
        telemetry.addLine("Left bumper: precision mode");
        telemetry.addLine("B button: Field/Robot-centric toggle");
        telemetry.update();
    }

    @Override
    public void start() {
        // true = request brake mode when teleop starts.
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        updateToggles();

        // FTC gamepad Y axis is positive when pushed down, so invert it for forward.
        double forward = shapeInput(-gamepad1.left_stick_y);
        double strafe = shapeInput(-gamepad1.left_stick_x);
        double turn = shapeInput(-gamepad1.right_stick_x);

        if (slowMode) {
            forward *= SLOW_MODE_MULTIPLIER;
            strafe *= SLOW_MODE_MULTIPLIER;
            turn *= TURN_SLOW_MODE_MULTIPLIER;
        }

        // Pedro: false = Field-Centric, true = Robot-Centric.
        follower.setTeleOpDrive(
                forward,
                strafe,
                turn,
                robotCentric
        );

        follower.update();

        updateTelemetry();
    }

    /**
     * Converts a raw joystick value into a cleaner competition input.
     * The deadband removes tiny stick noise and the cubic curve gives more
     * resolution around zero without reducing maximum power.
     */
    private double shapeInput(double value) {
        value = applyDeadband(value, JOYSTICK_DEADBAND);
        return value * value * value;
    }

    private double applyDeadband(double value, double deadband) {
        double magnitude = Math.abs(value);

        if (magnitude <= deadband) {
            return 0.0;
        }

        // Rescale so the first non-zero value starts at 0 and full stick stays 1.
        double scaled = (magnitude - deadband) / (1.0 - deadband);
        return Math.copySign(scaled, value);
    }

    private void updateToggles() {
        // Left bumper toggles precision mode.
        boolean slowButton = gamepad1.left_bumper;
        if (slowButton && !lastSlowButton) {
            slowMode = !slowMode;
        }
        lastSlowButton = slowButton;

        // B toggles between Field-Centric and Robot-Centric.
        boolean modeButton = gamepad1.b;
        if (modeButton && !lastModeButton) {
            robotCentric = !robotCentric;
        }
        lastModeButton = modeButton;
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addData("Drive Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
        telemetry.addData("Precision", slowMode ? "ON" : "OFF");

        telemetry.addData("X", "%.2f in", pose.getX());
        telemetry.addData("Y", "%.2f in", pose.getY());
        telemetry.addData("Heading", "%.2f deg", Math.toDegrees(pose.getHeading()));

        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Busy", follower.isBusy());

        telemetry.update();
    }
}
