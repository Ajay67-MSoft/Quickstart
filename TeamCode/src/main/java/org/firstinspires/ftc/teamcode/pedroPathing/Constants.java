package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients; // Added for true Predictive Braking
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.7006) // Weight in KG


            // === TRUE PREDICTIVE BRAKING COEFFICIENTS ===
            // Setting this automatically activates the real predictive braking algorithm.
            // Values: (kP, kLinear, kQuadratic)
            // Use PredictiveBrakingTuner to get kLinear and kQuadratic, then tune kP manually.
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    0.17,               // kP
                    0.05568320676302328,          // kLinear
                    0.0017463909197535704         // kQuadratic
            ))

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.07, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0, .007, .06, .01))


            // Turn off centripetal corrections since true predictive braking accounts for momentum
            .centripetalScaling(0.0)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightWheelDS")
            .rightRearMotorName("backRightWheelDS")
            .leftRearMotorName("backLeftWheelDS")
            .leftFrontMotorName("frontLeftWheelDS")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(84.019)
            .yVelocity(68.822);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.5)
            .strafePodX(-7.55)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    // Default path thresholds
    public static PathConstraints pathConstraints = new PathConstraints(
            0.9,
            100,
            1.3,
            0.5);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}



