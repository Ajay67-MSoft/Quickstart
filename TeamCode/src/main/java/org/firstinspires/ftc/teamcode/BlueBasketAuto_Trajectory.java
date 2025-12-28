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
 * This autonomous OpMode follows the linear path defined in BlueBasketAuto_Trajectory.pp.
 */
@Autonomous(name = "Blue Basket Auto", group = "Autonomous")
public class BlueBasketAuto_Trajectory extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Start pose from BlueBasketAuto_Trajectory.pp
    private final Pose startPose = new Pose(21.588, 121.463, Math.toRadians(90));

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

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
            case 0: // Path 1: Move to scoring position with linear heading transition
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(21.588, 121.463, Math.toRadians(90)), new Pose(62.155, 89.199, Math.toRadians(180))))
                        .setTangentHeadingInterpolatio
                        .build());
                setPathState(1);
                break;

            case 1: // Wait 2: 500ms
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(2);
                }
                break;

            case 2: // Path 3: Linear motion to next sample
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(62.155, 89.199, follower.getPose().getHeading()), new Pose(38.669, 83.506, Math.toRadians(180))))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
                setPathState(3);
                break;

            case 3: // Wait 4
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(4);
                }
                break;

            case 4: // Path 5: Linear motion to scoring
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(38.669, 83.506, follower.getPose().getHeading()), new Pose(61.680, 88.488, Math.toRadians(180))))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
                setPathState(5);
                break;

            case 5: // Wait 6
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(6);
                }
                break;

            case 6: // Path 7: Linear motion to next sample
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(61.680, 88.488, follower.getPose().getHeading()), new Pose(38.669, 60.020, Math.toRadians(180))))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
                setPathState(7);
                break;

            case 7: // Wait 8
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(8);
                }
                break;

            case 8: // Path 9: Linear motion to scoring
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(38.669, 60.020, follower.getPose().getHeading()), new Pose(62.155, 88.725, Math.toRadians(180))))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
                setPathState(9);
                break;

            case 9: // Wait 10
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(10);
                }
                break;

            case 10: // Path 11: Linear motion to next sample
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(62.155, 88.725, follower.getPose().getHeading()), new Pose(37.957, 35.585, Math.toRadians(180))))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
                setPathState(11);
                break;

            case 11: // Wait 12
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(12);
                }
                break;

            case 12: // Path 13: Linear motion to scoring
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(37.957, 35.585, follower.getPose().getHeading()), new Pose(62.629, 89.199, Math.toRadians(180))))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
                setPathState(13);
                break;

            case 13: // Wait 14
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(14);
                }
                break;

            case 14: // Path 15: Final linear motion to park/collect
                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(62.629, 89.199, follower.getPose().getHeading()), new Pose(38.194, 33.924, Math.toRadians(180))))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
                setPathState(15);
                break;

            case 15: // Wait 16
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }
}