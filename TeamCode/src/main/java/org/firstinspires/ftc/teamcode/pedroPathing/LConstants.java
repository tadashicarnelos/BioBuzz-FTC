package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * goBILDA Pinpoint localization configuration.
 *
 * The two offsets are intentionally left at zero until the robot's actual
 * pod locations are measured/tuned. Incorrect offsets can make heading and
 * position estimates worse, so they should not be guessed from a photograph.
 */
public class LConstants {

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )

            // Provisional only: run Pedro's automatic offset tuner and replace these.
            .forwardPodY(0.0)
            .strafePodX(0.0)

            // Verify with Localization Test: forward -> X increases, left -> Y increases.
            .forwardEncoderDirection(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            )
            .strafeEncoderDirection(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
}
