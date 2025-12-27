package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BlueStructureStartingPoint2 extends OpMode {

    /* ================= HARDWARE ================= */

    private DcMotor leftFlywheel;
    private DcMotor rightFlywheel;

    private Servo finalIntakeLeft;
    private Servo finalIntakeRight;

    /* ================= PEDRO ================= */

    private Follower follower;
    private Timer stateTimer;

    /* ================= STATES ================= */

    public enum State {
        DRIVE_TO_SHOOT_1,
        SHOOT_1,
        DRIVE_TO_COLLECT,
        COLLECT_1,
        COLLECT_2,
        COLLECT_3,
        DRIVE_BACK_TO_SHOOT,
        FINISHED
    }

    private State state;

    /* ================= POSES ================= */

    private final Pose startPose = new Pose(21.04, 123.35, Math.toRadians(144));
    private final Pose shootPose = new Pose(64.5, 98.0, Math.toRadians(142));

    private final Pose collect1 = new Pose(40.4, 76, Math.toRadians(180));
    private final Pose collect2 = new Pose(34.9, 76, Math.toRadians(180));
    private final Pose collect3 = new Pose(30.0, 76, Math.toRadians(180));
    private final Pose collect4 = new Pose(25.0, 76, Math.toRadians(180));

    /* ================= PATHS ================= */

    private PathChain pathShoot1;
    private PathChain pathCollect1;
    private PathChain pathCollect2;
    private PathChain pathCollect3;
    private PathChain pathReturnShoot;

    /* ================= INIT ================= */

    @Override
    public void init() {

        /* ---- Pedro ---- */
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        follower.setMaxPower(0.70); // ------------ 70% of max power --------------

        /* ---- Timers ---- */
        stateTimer = new Timer();

        /* ---- Motors ---- */
        leftFlywheel  = hardwareMap.get(DcMotor.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotor.class, "6000 RPM motor flywheel right");

        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /* ---- Servos ---- */
        finalIntakeLeft  = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(Servo.class, "finalIntakeServo");

        finalIntakeRight.setDirection(Servo.Direction.REVERSE);

        // Default: NOT feeding
        finalIntakeLeft.setPosition(20.0);
        finalIntakeRight.setPosition(20.0);

        buildPaths();

        state = State.DRIVE_TO_SHOOT_1;
        stateTimer.resetTimer();
    }

    /* ================= PATH BUILDING ================= */

    private void buildPaths() {

        pathShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pathCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1.getHeading())
                .build();

        pathCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1, collect2))
                .setLinearHeadingInterpolation(collect1.getHeading(), collect2.getHeading())
                .build();

        pathCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(collect2, collect3))
                .setLinearHeadingInterpolation(collect2.getHeading(), collect3.getHeading())
                .build();

        pathReturnShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect4, shootPose))
                .setLinearHeadingInterpolation(collect4.getHeading(), shootPose.getHeading())
                .build();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {

        follower.update();
        updateStateMachine();

        telemetry.addData("State", state);
        telemetry.addData("Timer", stateTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    /* ================= STATE MACHINE ================= */

    private void updateStateMachine() {

        switch (state) {

            case DRIVE_TO_SHOOT_1:
                follower.followPath(pathShoot1, true);
                transition(State.SHOOT_1);
                break;

            case SHOOT_1:
                if (!follower.isBusy()) {

                    double t = stateTimer.getElapsedTimeSeconds();

                    // Spin flywheels (MATCHES TELEOP)
                    leftFlywheel.setPower(-0.635);
                    rightFlywheel.setPower(0.635);

                    // Feed after 5 seconds
                    if (t > 5.0) {
                        finalIntakeLeft.setPosition(0.0);
                        finalIntakeRight.setPosition(0.0);

                        transition(State.DRIVE_TO_COLLECT);
                    }
                }
                break;

            case DRIVE_TO_COLLECT:
                stopShooter();
                follower.followPath(pathCollect1, true);
                transition(State.COLLECT_1);
                break;

            case COLLECT_1:
                if (!follower.isBusy()) {
                    follower.followPath(pathCollect2, true);
                    transition(State.COLLECT_2);
                }
                break;

            case COLLECT_2:
                if (!follower.isBusy()) {
                    follower.followPath(pathCollect3, true);
                    transition(State.COLLECT_3);
                }
                break;

            case COLLECT_3:
                if (!follower.isBusy()) {
                    follower.followPath(pathReturnShoot, true);
                    transition(State.DRIVE_BACK_TO_SHOOT);
                }
                break;

            case DRIVE_BACK_TO_SHOOT:
                if (!follower.isBusy()) {
                    transition(State.FINISHED);
                }
                break;

            case FINISHED:
                stopShooter();
                break;
        }
    }

    /* ================= HELPERS ================= */

    private void transition(State next) {
        state = next;
        stateTimer.resetTimer();
    }

    private void stopShooter() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
        finalIntakeLeft.setPosition(1.0);
        finalIntakeRight.setPosition(1.0);
    }
}
