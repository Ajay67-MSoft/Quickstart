package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This autonomous OpMode follows the path defined in N_trajectory1.pp.
 */
@Autonomous(name = "Pedro Auto", group = "Autonomous")
public class PedroAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define the start pose from N_trajectory1.pp
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    /**
     * This method is run once when the "INIT" button is pressed on the Driver Station.
     */
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    /**
     * This method is run once when the "PLAY" button is pressed on the Driver Station.
     */
    @Override
    public void start() {
        setPathState(0);
    }

    /**
     * This method is run continuously after "PLAY" is pressed until the OpMode is stopped.
     */
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path 1: Move to (23.8, 35.7) with heading 90 -> 180
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), new Pose(23.82, 35.73, Math.toRadians(180))))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                        .build());
                setPathState(1);
                break;

            case 1: // Wait 2: 1000ms
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) {
                    setPathState(2);
                }
                break;

            case 2: // Path 3: Move to (96.5, 95.7) with tangential heading
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(23.82, 35.73, follower.getPose().getHeading()), new Pose(96.56, 95.71, Math.toRadians(0))))
                        .setTangentHeadingInterpolation()
                        .build());
                setPathState(3);
                break;

            case 3: // Wait 4 & 5: ~5000ms total
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5005) {
                    setPathState(4);
                }
                break;

            case 4: // Path 6: Move to (23.1, 59.5) with tangential heading
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(96.56, 95.71, follower.getPose().getHeading()), new Pose(23.18, 59.55, Math.toRadians(0))))
                        .setTangentHeadingInterpolation()
                        .build());
                setPathState(5);
                break;

            case 5: // Wait 7: 1000ms
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) {
                    setPathState(6);
                }
                break;

            case 6: // Path 8: Move to (96.3, 95.2) with constant heading 45
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(23.18, 59.55, follower.getPose().getHeading()), new Pose(96.35, 95.29, Math.toRadians(45))))
                        .setConstantHeadingInterpolation(Math.toRadians(45))
                        .build());
                setPathState(7);
                break;

            case 7: // Wait 9: 5000ms
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5000) {
                    setPathState(8);
                }
                break;

            case 8: // Path 10: Move to (23.6, 83.8) with tangential heading
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(96.35, 95.29, follower.getPose().getHeading()), new Pose(23.61, 83.80, Math.toRadians(0))))
                        .setTangentHeadingInterpolation()
                        .build());
                setPathState(9);
                break;

            case 9: // Wait 11: 1000ms
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) {
                    setPathState(10);
                }
                break;

            case 10: // Path 12: Move to (96.5, 95.0) with constant heading 45
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(23.61, 83.80, follower.getPose().getHeading()), new Pose(96.56, 95.07, Math.toRadians(45))))
                        .setConstantHeadingInterpolation(Math.toRadians(45))
                        .build());
                setPathState(11);
                break;

            case 11: // Wait 13: 5000ms
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5000) {
                    setPathState(12);
                }
                break;

            case 12: // Path 14: Move to (39.7, 34.2) with heading 45 -> 90
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(96.56, 95.07, Math.toRadians(45)), new Pose(39.77, 34.24, Math.toRadians(90))))
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                        .build());
                setPathState(13);
                break;

            case 13: // Wait 15: 1000ms
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }
}
