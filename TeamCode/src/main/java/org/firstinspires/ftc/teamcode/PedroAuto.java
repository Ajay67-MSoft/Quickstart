package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This is an example autonomous OpMode using Pedro Pathing.
 * It uses the Follower and PathChain to move the robot along a path.
 */
@Autonomous(name = "Pedro Auto", group = "Autonomous")
public class PedroAuto extends OpMode {

    private Follower follower;

    // Define your start pose
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Define your PathChain
    private PathChain autoPath;

    /**
     * This method is run once when the "INIT" button is pressed on the Driver Station.
     */
    @Override
    public void init() {
        // Initialize the follower using the Constants class
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    /**
     * This method is run once when the "PLAY" button is pressed on the Driver Station.
     */
    @Override
    public void start() {
        // Build the path. You can copy the coordinates from the Pedro Pathing visualizer here.
        // Example path: Move forward 24 inches and turn 90 degrees
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, Math.toRadians(0)), new Pose(24, 0, Math.toRadians(0))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Pose(24, 0, Math.toRadians(0)), new Pose(24, 24, Math.toRadians(90))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Start following the path
        follower.followPath(autoPath);
    }

    /**
     * This method is run continuously after "PLAY" is pressed until the OpMode is stopped.
     */
    @Override
    public void loop() {
        // Update the follower to maintain the path
        follower.update();

        // Feedback to Driver Station
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.update();
    }
}
