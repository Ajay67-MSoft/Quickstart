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

/*
GOALS WITH THIS COMMIT
1. fix flywheels starting immediately
2. readjust values based on blue structure instead of yellow line
 */


@TeleOp
public class BlueStructureStartingPoint2 extends OpMode {

    private boolean pathStarted = false;


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

    private final Pose startPose = new Pose(24.746955345060893, 128.60622462787552, Math.toRadians(143)); // fix 2
    private final Pose shootPose = new Pose(64.5, 98.0, Math.toRadians(142)); // fix 2

    private final Pose collect1 = new Pose(40.4, 83, Math.toRadians(180)); // fix 2
    private final Pose collect2 = new Pose(34.9, 83, Math.toRadians(180)); // fix 2
    private final Pose collect3 = new Pose(30.0, 83, Math.toRadians(180)); // fix 2

    /* ================= PATHS ================= */

    private PathChain pathShoot1;
    private PathChain pathCollect1;
    private PathChain pathCollect2;
    private PathChain pathCollect3;
    private PathChain pathReturnShoot;

    /* ================= SERVO POSITIONS ================= */

    // defines final intake servo feeding and default positions
    private final double SERVO_FEED_POSITION = 0.0;    // Position to feed game elements
    private final double SERVO_STOP_POSITION = 20;    // Default position

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

        // Default: NOT feeding (valid servo range is 0.0 to 1.0)
        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
        finalIntakeRight.setPosition(SERVO_STOP_POSITION);

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
                .addPath(new BezierLine(collect3, shootPose))
                .setLinearHeadingInterpolation(collect3.getHeading(), shootPose.getHeading())
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
        telemetry.addData("Left Flywheel Power", leftFlywheel.getPower());
        telemetry.addData("Right Flywheel Power", rightFlywheel.getPower());
        telemetry.addData("Servo Left Pos", finalIntakeLeft.getPosition());
        telemetry.addData("Servo Right Pos", finalIntakeRight.getPosition());
        telemetry.update();
    }

    /* ================= STATE MACHINE ================= */

    private void updateStateMachine() {

        switch (state) {

            case DRIVE_TO_SHOOT_1:
                if (!pathStarted) {
                    follower.followPath(pathShoot1, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    transition(State.SHOOT_1);
                }
                break;

            case SHOOT_1:
                double t = stateTimer.getElapsedTimeSeconds();

                // fly wheel
                leftFlywheel.setPower(-0.635);
                rightFlywheel.setPower(0.635);

                // Wait for flywheels to spin up, then feed
                if (t > 5.0) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (t > 6.0) {
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                    finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                    finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                }

                // After feeding for 1 second, move to next state
                if (t > 7.0) {
                    transition(State.DRIVE_TO_COLLECT);
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
        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
        finalIntakeRight.setPosition(SERVO_STOP_POSITION);
    }
}