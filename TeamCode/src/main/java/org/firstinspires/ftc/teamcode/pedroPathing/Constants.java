package org.firstinspires.ftc.teamcode.pedroPathing;
//import org.firstinspires.ftc.teamcode.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .mass(10.44) // Weight in KG (10.44 lbs = 4.73 kg)
            .forwardZeroPowerAcceleration(-30.596899568015857)
            .lateralZeroPowerAcceleration(-51.04545201410181)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0.022,0.004))
            .headingPIDFCoefficients(new PIDFCoefficients(.65,0,0.05,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0,.007,.06,.01))
            .centripetalScaling(-.01)
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
            .xVelocity(66.53458386518824)
            .yVelocity(55.39393051027314);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.5)
            .strafePodX(-7.55)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            //.customEncoderResolution(4000/(2*Math.PI*16))
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.9,
            100,
            .85,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
