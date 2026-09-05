package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Central Pedro Pathing configuration and Follower factory.
 */
public class Constants {

    // These are intentionally conservative until the robot is tuned.
    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    /**
     * Creates the Pedro Follower with our Mecanum drivetrain and Pinpoint localizer.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FConstants.followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(FConstants.driveConstants)
                .pinpointLocalizer(LConstants.localizerConstants)
                .build();
    }
}
