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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class REDstructureStartingPoint extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake1150;

    private Servo finalIntakeLeft;
    private Servo finalIntakeRight;

    /* ================= SHOOTING CONSTANTS ================= */
    private boolean shootLeft = false;
    private boolean shootRight = false;

    private int shootFirst = 500;
    private int prepareSecond = 1500;
    private int stopIntake = 3500;
    private int shootSecond = 4500;

    private double SHOOT_RPM = 2500;
    private double TARGET_SHOOT_RPM = 2500;
    private static final double TICKS_PER_REV = 28.0;
    private final double shootTicksPerSec = SHOOT_RPM * TICKS_PER_REV / 60.0;
    private static final double RPM_TOLERANCE = 100;

    private int IntakeInward = -1;

    private ElapsedTime timerLeft = new ElapsedTime();
    private ElapsedTime timerRight = new ElapsedTime();

    /* ================= PEDRO ================= */
    private Follower follower;
    private Timer stateTimer;

    /* ================= STATES ================= */
    public enum State {
        DRIVE_TO_SHOOT_1,
        SHOOT_1,
        DRIVE_TO_COLLECT_1,
        COLLECT_1,
        COLLECT_2,
        DRIVE_BACK_TO_SHOOT_2,
        SHOOT_2,
        DRIVE_OUTSIDE,
        FINISHED
    }

    private State state;

    /* ================= POSES (FIXED RED MIRROR) ================= */
    private final Pose startPose =
            new Pose(119.253044655 + 8, 128.60622462787552, Math.toRadians(37));

    private final Pose shootPose =
            new Pose(92.5575101488498, 104.83355886332882, Math.toRadians(35));

    private final Pose collect1 =
            new Pose(99.6, 84, Math.toRadians(0));

    private final Pose collect2 =
            new Pose(129.0, 80, Math.toRadians(0));

    private final Pose shootPose2 =
            new Pose(92.5575101488498, 104.83355886332882, Math.toRadians(42.5));

    private final Pose endPose =
            new Pose(99.6 - 8, 72, Math.toRadians(0));

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

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        intake1150 = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        finalIntakeLeft = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(Servo.class, "finalIntakeServo");
        finalIntakeRight.setDirection(Servo.Direction.REVERSE);

        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
        finalIntakeRight.setPosition(SERVO_STOP_POSITION);

        leftFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients shooterPIDF =
                new PIDFCoefficients(0.011, 0.0, 0.001, 14.6);

        leftFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        rightFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        buildPaths();

        state = State.DRIVE_TO_SHOOT_1;
        stateTimer.resetTimer();
    }

    /* ================= PATH BUILDING ================= */
    private void buildPaths() {

        pathShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(), shootPose.getHeading())
                .build();

        pathCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(), collect1.getHeading())
                .build();

        pathCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1, collect2))
                .setLinearHeadingInterpolation(
                        collect1.getHeading(), collect2.getHeading())
                .build();

        pathReturnShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect2, shootPose2))
                .setLinearHeadingInterpolation(
                        collect2.getHeading(), shootPose2.getHeading())
                .build();

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, endPose))
                .setLinearHeadingInterpolation(
                        shootPose2.getHeading(), endPose.getHeading())
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

        double leftRPM = leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        double rightRPM = rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;

        telemetry.addData("Flywheel RPM Left", leftRPM);
        telemetry.addData("Flywheel RPM Right", rightRPM);
        telemetry.update();

        switch (state) {

            case DRIVE_TO_SHOOT_1:
                leftFlywheel.setVelocity(-shootTicksPerSec);
                rightFlywheel.setVelocity(shootTicksPerSec);

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
                leftFlywheel.setVelocity(-shootTicksPerSec);
                rightFlywheel.setVelocity(shootTicksPerSec);

                if (stateTimer.getElapsedTimeSeconds() >= 8) {
                    transition(State.DRIVE_TO_COLLECT_1);
                    follower.followPath(pathCollect1);
                }

                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE && !shootLeft) {
                    shootLeft = true;
                    timerLeft.reset();
                }
                if (Math.abs(rightRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE && !shootRight) {
                    shootRight = true;
                    timerRight.reset();
                }

                handleShooting();
                break;

            case DRIVE_TO_COLLECT_1:
                intake1150.setPower(IntakeInward);
                if (!follower.isBusy()) {
                    follower.followPath(pathCollect1, true);
                    transition(State.COLLECT_1);
                }
                break;

            case COLLECT_1:
                if (!follower.isBusy()) {
                    follower.followPath(pathCollect2, true);
                    transition(State.COLLECT_2);
                }
                break;

            case COLLECT_2:
                if (!follower.isBusy()) {
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
                leftFlywheel.setVelocity(-shootTicksPerSec);
                rightFlywheel.setVelocity(shootTicksPerSec);

                if (stateTimer.getElapsedTimeSeconds() >= 8) {
                    transition(State.DRIVE_OUTSIDE);
                    follower.followPath(pathDriveToEnd);
                }

                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE && !shootLeft) {
                    shootLeft = true;
                    timerLeft.reset();
                }
                if (Math.abs(rightRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE && !shootRight) {
                    shootRight = true;
                    timerRight.reset();
                }

                handleShooting();
                break;

            case DRIVE_OUTSIDE:
                if (!follower.isBusy()) {
                    transition(State.FINISHED);
                }
                break;

            case FINISHED:
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
                intake1150.setPower(0);
                finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                break;
        }
    }

    /* ================= SHOOT LOGIC ================= */
    private void handleShooting() {

        if (shootLeft) {
            long t = (long) timerLeft.milliseconds();

            if (t < shootFirst) {
                finalIntakeLeft.setPosition(0);
            } else if (t < prepareSecond) {
                finalIntakeLeft.setPosition(20);
                intake1150.setPower(IntakeInward);
            } else if (t < stopIntake) {
                intake1150.setPower(0);
            } else if (t < shootSecond) {
                finalIntakeLeft.setPosition(0);
            } else {
                finalIntakeLeft.setPosition(20);
                shootLeft = false;
            }
        }

        if (shootRight) {
            long t = (long) timerRight.milliseconds();

            if (t < shootFirst) {
                finalIntakeRight.setPosition(0);
            } else if (t < prepareSecond) {
                finalIntakeRight.setPosition(20);
                intake1150.setPower(IntakeInward);
            } else if (t < stopIntake) {
                intake1150.setPower(0);
            } else if (t < shootSecond) {
                finalIntakeRight.setPosition(0);
            } else {
                finalIntakeRight.setPosition(20);
                shootRight = false;
            }
        }
    }

    /* ================= HELPERS ================= */
    private void transition(State next) {
        state = next;
        stateTimer.resetTimer();
    }
}
