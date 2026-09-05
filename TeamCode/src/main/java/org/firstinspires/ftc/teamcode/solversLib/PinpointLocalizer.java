package org.firstinspires.ftc.teamcode.solversLib;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Small adapter around the GoBILDA Pinpoint so SolversLib can use its heading
 * for field-centric mecanum driving.
 *
 * Pinpoint itself performs the two-dead-wheel + IMU localization. SolversLib
 * handles the mecanum drive and field-centric transformation.
 */
public class PinpointLocalizer {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(
                DriveConstants.PINPOINT_X_OFFSET_MM,
                DriveConstants.PINPOINT_Y_OFFSET_MM
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Verify these directions on the real robot during tuning.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public double getHeadingDegrees() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }

    public double getXInches() {
        return getPose().getX(DistanceUnit.INCH);
    }

    public double getYInches() {
        return getPose().getY(DistanceUnit.INCH);
    }
}
