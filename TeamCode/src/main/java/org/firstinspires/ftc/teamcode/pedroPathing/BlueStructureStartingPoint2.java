package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class BlueStructureStartingPoint2 extends OpMode {

    /* ================= HARDWARE ================= */

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake1150;

    private Servo finalIntakeLeft;
    private Servo finalIntakeRight;

    /* ================= FLYWHEEL RPM CONTROL ================= */

    private static final double TICKS_PER_REV = 28.0; // REV HD Hex
    private static final double TARGET_RPM = 2000.0;
    private static final double RPM_TOLERANCE = 75.0;
    private static final double kP = 0.00035;

    private double flywheelPower = 0.35;

    /* ================= PEDRO ================= */

    private Follower follower;
    private Timer stateTimer;
    private boolean pathStarted = false;

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

    private final Pose startPose = new Pose(24.746955345060893, 128.60622462787552, Math.toRadians(143));
    private final Pose shootPose = new Pose(51.4424898511502, 104.83355886332882, Math.toRadians(143));
    private final Pose collect1 = new Pose(44.4, 84, Math.toRadians(180));
    private final Pose collect2 = new Pose(29, 84, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(51.4424898511502, 104.83355886332882, Math.toRadians(138));
    private final Pose endPose = new Pose(44.4, 72, Math.toRadians(180));

    /* ================= PATHS ================= */

    private PathChain pathShoot1;
    private PathChain pathCollect1;
    private PathChain pathCollect2;
    private PathChain pathReturnShoot;
    private PathChain pathDriveToEnd;

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

        leftFlywheel  = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        intake1150    = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        finalIntakeLeft  = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(Servo.class, "finalIntakeServo");
        finalIntakeRight.setDirection(Servo.Direction.REVERSE);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopFlywheel();
        intake1150.setPower(0);

        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
        finalIntakeRight.setPosition(SERVO_STOP_POSITION);

        telemetry.addLine("14 POOPS ON EILEEN");

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

        pathReturnShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect2, shootPose2))
                .setLinearHeadingInterpolation(collect2.getHeading(), shootPose2.getHeading())
                .build();

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, endPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), endPose.getHeading())
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
                finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                finalIntakeRight.setPosition(SERVO_STOP_POSITION);

                boolean ready1 = updateFlywheelRPM();

                if (ready1 && stateTimer.getElapsedTimeSeconds() > 0.25) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (ready1 && stateTimer.getElapsedTimeSeconds() > 1.25) {
                    intake1150.setPower(-1);
                }

                if (ready1 && stateTimer.getElapsedTimeSeconds() > 2.25) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (ready1 && stateTimer.getElapsedTimeSeconds() > 3.25) {
                    stopFlywheel();
                    intake1150.setPower(0);
                    transition(State.DRIVE_TO_COLLECT);
                }
                break;

            case DRIVE_TO_COLLECT:
                follower.followPath(pathCollect1, true);
                transition(State.COLLECT_1);
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
                    intake1150.setPower(0);
                    follower.followPath(pathReturnShoot, true);
                    transition(State.DRIVE_BACK_TO_SHOOT_2);
                }
                break;

            case DRIVE_BACK_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    transition(State.SHOOT_2);
                }
                break;

            case SHOOT_2:
                finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                finalIntakeRight.setPosition(SERVO_STOP_POSITION);

                boolean ready2 = updateFlywheelRPM();

                if (ready2 && stateTimer.getElapsedTimeSeconds() > 0.25) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (ready2 && stateTimer.getElapsedTimeSeconds() > 1.25) {
                    intake1150.setPower(-1);
                }

                if (ready2 && stateTimer.getElapsedTimeSeconds() > 2.25) {
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                if (ready2 && stateTimer.getElapsedTimeSeconds() > 3.75) {
                    stopFlywheel();
                    intake1150.setPower(0);
                    follower.followPath(pathDriveToEnd, true);
                    transition(State.DRIVE_OUTSIDE);
                }
                break;

            case DRIVE_OUTSIDE:
                if (!follower.isBusy()) {
                    transition(State.FINISHED);
                }
                break;

            case FINISHED:
                stopFlywheel();
                intake1150.setPower(0);
                finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                break;
        }
    }

    /* ================= HELPERS ================= */

    private boolean updateFlywheelRPM() {
        double leftRPM  = Math.abs((leftFlywheel.getVelocity() / TICKS_PER_REV) * 60.0);
        double rightRPM = Math.abs((rightFlywheel.getVelocity() / TICKS_PER_REV) * 60.0);
        double avgRPM   = (leftRPM + rightRPM) / 2.0;

        double error = TARGET_RPM - avgRPM;
        flywheelPower += error * kP;
        flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));

        leftFlywheel.setPower(-flywheelPower);
        rightFlywheel.setPower(flywheelPower);

        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("RPM", avgRPM);
        telemetry.addData("Power", flywheelPower);

        return Math.abs(error) <= RPM_TOLERANCE;
    }

    private void stopFlywheel() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }

    private void transition(State next) {
        state = next;
        stateTimer.resetTimer();
    }
}
