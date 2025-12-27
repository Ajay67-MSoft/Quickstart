package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

@TeleOp
public class BlueStructureStartingPoint extends OpMode {
    private PathChain blueStructurePath;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POS
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }

    PathState pathState;

    Pose[] points = new Pose[5];


//    private final Pose startPose = new Pose(21.044654939106902, 123.34506089309879, Math.toRadians(323));
//    private final Pose goToPickup

    @Override
    public void init() {
        points[0] = new Pose(21.044654939106902, 123.34506089309879, Math.toRadians(323));
        points[1] = new Pose(40.4, 85.6, Math.toRadians(180));
        points[2] = new Pose(34.9, 85.6, Math.toRadians(180));
        points[3] = new Pose(30, 85.6, Math.toRadians(180));
        points[4] = new Pose(25, 85.6, Math.toRadians(180));
        points[5] = new Pose(64.4979702300406, 98.40324763193506, Math.toRadians(142));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly unabstractastracta not shape, starting on the apple point.");
        telemetryM.debug("So, make sure you have enough space to the up, down and outside to run the OpMode.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void loop() {
        follower.update();
        draw();

        if (follower.atParametricEnd()) {
            follower.followPath(blueStructurePath, true);
        }
    }

    @Override
    public void start() {
        PathBuilder pathBuilder = follower.pathBuilder();
//        blueStructurePath = follower.pathBuilder();
        for (int i = 0; i < points.length - 1; i++) {
            pathBuilder.addPath(new BezierLine(points[i], points[i + 1]))
            .setLinearHeadingInterpolation(points[i].getHeading(), points[i + 1].getHeading());
        }
//        pathBuilder.build();
        follower.followPath(pathBuilder.build());
    }
}
