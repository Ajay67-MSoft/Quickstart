package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 GOALS WITH THIS COMMIT
 1. implement motor encoders
 */

@Autonomous
public class BlueStructureStartingPoint2 extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */
    private DcMotor leftFlywheel;
    private DcMotor rightFlywheel;
    private DcMotor intake1150;

    private Servo finalIntakeLeft;
    private Servo finalIntakeRight;

    private double leftFlywheelPower = -0.58 + 0.05;   // original was 0.58
    private double rightFlywheelPower = 0.5 - 0.05;    // original was 0.5
    private double flywheelRampUpDurationSeconds = 3.0;

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
        DRIVE_BACK_TO_SHOOT_2,
        SHOOT_2,
        DRIVE_OUTSIDE,
        FINISHED
    }

    private State state;

    /* ================= POSES ================= */
    private final Pose startPose = new Pose(
            24.746955345060893,
            128.60622462787552,
            Math.toRadians(143)
    );

    private final Pose shootPose = new Pose(
            51.4424898511502,
            104.83355886332882,
            Math.toRadians(143)
    );

    private final Pose collect1 = new Pose(44.4, 84, Math.toRadians(180));
    private final Pose collect2 = new Pose(29, 84, Math.toRadians(180));

    private final Pose shootPose2 = new Pose(
            51.4424898511502,
            104.83355886332882,
            Math.toRadians(138)
    );

    private final Pose endPose = new Pose(44.4, 72, Math.toRadians(180));

    /* ================= PATHS ================= */
    private PathChain pathShoot1;
    private PathChain pathCollect1;
    private PathChain pathCollect2;
    private PathChain pathReturnShoot;
    private PathChain pathDriveToEnd;

    /* ================= SERVO POSITIONS ================= */
    private static final double SERVO_FEED_POSITION = 0.0;
    private static final double SERVO_STOP_POSITION = 20;

    /* ================= INIT ================= */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        follower.setMaxPower(0.70);

        stateTimer = new Timer();

        leftFlywheel = hardwareMap.get(DcMotor.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotor.class, "6000 RPM motor flywheel right");
        intake1150 = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1150.setPower(0);

        finalIntakeLeft = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(Servo.class, "finalIntakeServo");
        finalIntakeRight.setDirection(Servo.Direction.REVERSE);

        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
        finalIntakeRight.setPosition(SERVO_STOP_POSITION);

        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("14 POOPS ON EILEEN");

        buildPaths();

        state = State.DRIVE_TO_SHOOT_1;
        stateTimer.resetTimer();
    }

    /* ================= PATH BUILDING ================= */
    private void buildPaths() {

        pathShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        pathCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        collect1.getHeading()
                )
                .build();

        pathCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1, collect2))
                .setLinearHeadingInterpolation(
                        collect1.getHeading(),
                        collect2.getHeading()
                )
                .build();

        pathReturnShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect2, shootPose2))
                .setLinearHeadingInterpolation(
                        collect2.getHeading(),
                        shootPose2.getHeading()
                )
                .build();

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, endPose))
                .setLinearHeadingInterpolation(
                        shootPose2.getHeading(),
                        endPose.getHeading()
                )
                .build();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {
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

                leftFlywheel.setPower(leftFlywheelPower);
                rightFlywheel.setPower(rightFlywheelPower);

                if (t > flywheelRampUpDurationSeconds) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (t > flywheelRampUpDurationSeconds + 1.0) {
                    finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                    finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                }

                if (t > flywheelRampUpDurationSeconds + 2.0) {
                    intake1150.setPower(-1);
                }

                if (t > flywheelRampUpDurationSeconds + 3.0) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (t > flywheelRampUpDurationSeconds + 4.0) {
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                    intake1150.setPower(0);
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
                finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                intake1150.setPower(-1);

                if (!follower.isBusy()) {
                    follower.followPath(pathCollect2, true);
                    transition(State.COLLECT_2);
                }
                break;

            case COLLECT_2:
                if (!follower.isBusy()) {
                    follower.followPath(pathReturnShoot, true);
                    intake1150.setPower(0);
                    transition(State.DRIVE_BACK_TO_SHOOT_2);
                }
                break;

            case DRIVE_BACK_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    transition(State.SHOOT_2);
                }
                break;

            case SHOOT_2:
                double t2 = stateTimer.getElapsedTimeSeconds();

                finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                finalIntakeRight.setPosition(SERVO_STOP_POSITION);

                leftFlywheel.setPower(leftFlywheelPower);
                rightFlywheel.setPower(rightFlywheelPower);

                if (t2 > 3.0) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (t2 > 4.0) {
                    finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                    finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                }

                if (t2 > 5.0) {
                    intake1150.setPower(-1);
                }

                if (t2 > 6.0) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (t2 > 9.0) {
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                    intake1150.setPower(0);
                    follower.followPath(pathDriveToEnd, true);
                    transition(State.DRIVE_OUTSIDE);
                }
                break;

            case DRIVE_OUTSIDE:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
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
