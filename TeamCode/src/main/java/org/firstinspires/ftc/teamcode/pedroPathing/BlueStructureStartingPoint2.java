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
1. get shoot position right
2. get ball collect position right
 */

@TeleOp
public class BlueStructureStartingPoint2 extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */

    private DcMotor leftFlywheel;
    private DcMotor rightFlywheel;
    private DcMotor intake1150;

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

    private final Pose startPose = new Pose(24.746955345060893, 128.60622462787552, Math.toRadians(143));
    private final Pose shootPose = new Pose(51.4424898511502, 104.83355886332882, Math.toRadians(143));

    private final Pose collect1 = new Pose(40.4, 87, Math.toRadians(180));
    private final Pose collect2 = new Pose(34.9, 87, Math.toRadians(180));
    private final Pose collect3 = new Pose(30.0, 87, Math.toRadians(180));

    /* ================= PATHS ================= */

    private PathChain pathShoot1;
    private PathChain pathCollect1;
    private PathChain pathCollect2;
    private PathChain pathCollect3;
    private PathChain pathReturnShoot;

    /* ================= SERVO POSITIONS ================= */

    private final double SERVO_FEED_POSITION = 0.0;
    private final double SERVO_STOP_POSITION = 20;

    /* ================= INIT ================= */

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        follower.setMaxPower(0.70);

        stateTimer = new Timer();

        leftFlywheel  = hardwareMap.get(DcMotor.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotor.class, "6000 RPM motor flywheel right");
        intake1150    = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1150.setPower(0);

        finalIntakeLeft  = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(Servo.class, "finalIntakeServo");

        finalIntakeRight.setDirection(Servo.Direction.REVERSE);

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
        telemetry.addLine("POOPING"); // ------------ VERY IMPORTANT VERSION NUMBER LINE -----------
        follower.update();
        updateStateMachine();
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

                leftFlywheel.setPower(-0.6);
                rightFlywheel.setPower(0.58);

                if (t > 3.0) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (t > 4.0) {
                    leftFlywheel.setPower(0.6);
                    rightFlywheel.setPower(-0.58);
                }

                if (t > 4.75) {
                    finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                    finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                    intake1150.setPower(-1);
                    leftFlywheel.setPower(-0.6);
                    rightFlywheel.setPower(0.58);
                }

                if (t > 5) {
                    intake1150.setPower(0);
                }

                if (t > 7.75) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (t > 8.75) {
                    // stop flywheels
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                    // reset servo positions
                    finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                    finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                    transition(State.DRIVE_TO_COLLECT);
                }
                break;

            case DRIVE_TO_COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(pathCollect1, true);
                    transition(State.COLLECT_1);
                }

                break;

            case COLLECT_1:
                intake1150.setPower(-1);
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
                    intake1150.setPower(0);
                    follower.followPath(pathReturnShoot, true);
                    transition(State.DRIVE_BACK_TO_SHOOT);
                }
                break;

            case DRIVE_BACK_TO_SHOOT:
                double t2 = stateTimer.getElapsedTimeSeconds();

                if (t2 < 0.75) {
                    leftFlywheel.setPower(0.6);
                    rightFlywheel.setPower(-0.58);
                } else if (t2 < 1.0) {
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                } else if (t2 < 4.0) {
                    leftFlywheel.setPower(-0.6);
                    rightFlywheel.setPower(0.58);

                    if (t2 < 1.75) {
                        finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                        finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                    }
                } else {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                    transition(State.FINISHED);
                }
                break;

            case FINISHED:
                intake1150.setPower(0);
                leftFlywheel.setPower(0);
                rightFlywheel.setPower(0);
                finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                break;
        }
    }

    /* ================= HELPERS ================= */

    private void transition(State next) {
        state = next;
        stateTimer.resetTimer();
    }
}
