package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

// motor imports (kept)


@TeleOp
public class BlueStructureStartingPoint2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS1,
        SHOOTPOS_TOCOLLECTPOS,
        COLLECT_BALL1,
        COLLECT_BALL2,
        COLLECT_BALL3,
        TO_SHOOTPOS,
        FINISHED
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.044654939106902, 123.34506089309879, Math.toRadians(144));
    private final Pose interPose1 = new Pose(64.5, 98, Math.toRadians(142));
    private final Pose interPose2 = new Pose(40.4, 76, Math.toRadians(180));
    private final Pose interPose3 = new Pose(34.9, 76, Math.toRadians(180));
    private final Pose interPose4 = new Pose(30, 76, Math.toRadians(180));
    private final Pose interPose5 = new Pose(25, 76, Math.toRadians(180));
    private final Pose interPose6 = new Pose(64.4979702300406, 98.40324763193506, Math.toRadians(142));

    // add missing path declarations
    private PathChain path1, path2, path3, path4, path5, path6;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose1.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(interPose1, interPose2))
                .setLinearHeadingInterpolation(interPose1.getHeading(), interPose2.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(interPose2, interPose3))
                .setLinearHeadingInterpolation(interPose2.getHeading(), interPose3.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(interPose3, interPose4))
                .setLinearHeadingInterpolation(interPose3.getHeading(), interPose4.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(interPose4, interPose5))
                .setLinearHeadingInterpolation(interPose4.getHeading(), interPose5.getHeading())
                .build();
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(interPose5, interPose6))
                .setLinearHeadingInterpolation(interPose5.getHeading(), interPose6.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS1:
                follower.followPath(path1, true);
                setPathState(PathState.SHOOTPOS_TOCOLLECTPOS);
                break;
            case SHOOTPOS_TOCOLLECTPOS:
                follower.followPath(path2, true);
                setPathState(PathState.COLLECT_BALL1);
                break;

            case COLLECT_BALL1:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(PathState.COLLECT_BALL2);
                }
                break;

            case COLLECT_BALL2:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(PathState.COLLECT_BALL3);
                }
                break;

            case COLLECT_BALL3:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(PathState.TO_SHOOTPOS);
                }
                break;

            case TO_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(path6, true);
                    setPathState(PathState.FINISHED);
                }
                break;

            case FINISHED:
                // Do nothing — robot stops here
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS1;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
        follower.setMaxPower(0.70);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Current state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
/*
// terminal to run stuff
chmod: adb: No such file or directory
 % cd ~/Library/Android/sdk/platform-tools
 % ls
adb			hprof-conv		make_f2fs_casefold	NOTICE.txt		sqlite3
etc1tool		lib64			mke2fs			package.xml
fastboot		make_f2fs		mke2fs.conf		source.properties
 % chmod +x adb
 % ./adb devices
List of devices attached
4315Q2U9R9	device

 % ./adb connect 192.168.43.1:5555
connected to 192.168.43.1:5555
 */