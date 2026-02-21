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

/*
 GOALS WITH THIS COMMIT
 1. implement motor encoders
 */

@Autonomous
public class BlueStructureStartingPoint2 extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake1150;

    private Servo finalIntakeLeft;
    private Servo finalIntakeRight;



    /* ================= SHOOTING CONSTANTS ================= */

    // limelight ty: 16.5 - closest possible (in front of purple line) (2400-2500 RPM)
    private boolean shootLeft = false;
    private boolean shootRight = false;
    private int shootGap = 2000;
    private int shootFirst = 500;
    private int prepareSecond = 1500;
    private int stopIntake = 3500;
    private int shootSecond = 4500;
    private double SHOOT_RPM = 2500;
    private double TARGET_SHOOT_RPM = 2500;
    private static final double TICKS_PER_REV = 28.0;
    private final double shootTicksPerSec = SHOOT_RPM * TICKS_PER_REV / 60.0;;
    private static final double TARGET_RPM = 2000.0;
    private static final double RPM_TOLERANCE = 100;
    private int IntakeInward = -1;
    private int IntakeOutward = 1;
    private int IntakeNoPower = 0;

    private ElapsedTime timerLeft = new ElapsedTime();
    private ElapsedTime timerRight = new ElapsedTime();

    /* ================= PEDRO ================= */
    private Follower follower;
    private Timer stateTimer;

    /* ================= RPM TRACKING ================= */
    private int lastFlywheelPosition = 0;
    private double lastFlywheelTime = 0.0;

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

    /* ================= POSES ================= */
    private final Pose startPose = new Pose(24.746955345060893, 128.60622462787552, Math.toRadians(143));
    private final Pose shootPose = new Pose(51.4424898511502, 104.83355886332882, Math.toRadians(145));
    private final Pose collect1 = new Pose(44.4, 84, Math.toRadians(180));
    private final Pose collect2 = new Pose(15, 80, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(51.4424898511502, 104.83355886332882, Math.toRadians(137.5));
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

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        intake1150 = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1150.setPower(0);

        finalIntakeLeft = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(Servo.class, "finalIntakeServo");
        finalIntakeRight.setDirection(Servo.Direction.REVERSE);

        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
        finalIntakeRight.setPosition(SERVO_STOP_POSITION);

        leftFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lastFlywheelPosition = leftFlywheel.getCurrentPosition();
        lastFlywheelTime = stateTimer.getElapsedTimeSeconds();

        PIDFCoefficients shooterPIDF =
                new PIDFCoefficients(0.011, 0.0, 0.001, 14.6);

        leftFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        rightFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        telemetry.addLine("15 POOPS ON EILEEN");

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

        double currentRPM = getFlywheelRPM(leftFlywheel);

        double leftRPM = leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        double rightRPM = rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;

        telemetry.addData("Flywheel RPM Left:", "%.1f", leftRPM);
        telemetry.addData("Flywheel RPM Right:", "%.1f", rightRPM);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("timer: left", timerLeft.milliseconds());
        telemetry.addData("timer right:", timerRight.milliseconds());
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

                if (stateTimer.getElapsedTimeSeconds() >= 10) {
                    transition(State.DRIVE_TO_COLLECT_1);
                }

                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE
                        && !shootLeft) {
                    shootLeft = true;
                    timerLeft.reset();
                }
                if (Math.abs(rightRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE
                        && !shootRight) {
                    shootRight = true;
                    timerRight.reset();
                }

                if (shootLeft) {

                    if (timerLeft.milliseconds() < shootFirst) {  // 500 ms gap between this and above if is risky, if shooting isn't working change this
                        finalIntakeLeft.setPosition(0);
                    } else if (timerLeft.milliseconds() < prepareSecond) { // same comment as above
                        finalIntakeLeft.setPosition(20);
                        intake1150.setPower(IntakeInward);
                    } else if (timerLeft.milliseconds() < stopIntake) {
                        intake1150.setPower(0);
                    } else if (timerLeft.milliseconds() < shootSecond) { // same comment as above
                        finalIntakeLeft.setPosition(0);
                    } else {
                        finalIntakeLeft.setPosition(20);
                        leftFlywheel.setVelocity(0);
                        shootLeft = false;
                        timerLeft.reset();
                    }
                }
                if (shootRight) {

                    if (timerRight.milliseconds() < shootFirst) {  // 500 ms gap between this and above if is risky, if shooting isn't working change this
                        finalIntakeRight.setPosition(0);
                    } else if (timerRight.milliseconds() < prepareSecond) { // same comment as above
                        finalIntakeRight.setPosition(20);
                        intake1150.setPower(IntakeInward);
                    } else if (timerRight.milliseconds() < stopIntake) {
                        intake1150.setPower(0);
                    } else if (timerRight.milliseconds() < shootSecond) { // same comment as above
                        finalIntakeRight.setPosition(0);
                    } else {
                        finalIntakeRight.setPosition(20);
                        rightFlywheel.setVelocity(0);
                        shootRight = false;
                        timerRight.reset();
                    }
                }
                break;
            case SHOOT_2:

                leftFlywheel.setVelocity(-shootTicksPerSec);
                rightFlywheel.setVelocity(shootTicksPerSec);

                if (stateTimer.getElapsedTimeSeconds() >= 10) {
                    transition(State.DRIVE_OUTSIDE);
                    follower.followPath(pathDriveToEnd);
                }

                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE
                        && !shootLeft) {
                    shootLeft = true;
                    timerLeft.reset();
                }
                if (Math.abs(rightRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE
                        && !shootRight) {
                    shootRight = true;
                    timerRight.reset();
                }

                if (shootLeft) {

                    if (timerLeft.milliseconds() < shootFirst) {  // 500 ms gap between this and above if is risky, if shooting isn't working change this
                        finalIntakeLeft.setPosition(0);
                    } else if (timerLeft.milliseconds() < prepareSecond) { // same comment as above
                        finalIntakeLeft.setPosition(20);
                        intake1150.setPower(IntakeInward);
                    } else if (timerLeft.milliseconds() < stopIntake) {
                        intake1150.setPower(0);
                    } else if (timerLeft.milliseconds() < shootSecond) { // same comment as above
                        finalIntakeLeft.setPosition(0);
                    } else {
                        finalIntakeLeft.setPosition(20);
                        leftFlywheel.setVelocity(0);
                        shootLeft = false;
                        timerLeft.reset();
                    }
                }
                if (shootRight) {

                    if (timerRight.milliseconds() < shootFirst) {  // 500 ms gap between this and above if is risky, if shooting isn't working change this
                        finalIntakeRight.setPosition(0);
                    } else if (timerRight.milliseconds() < prepareSecond) { // same comment as above
                        finalIntakeRight.setPosition(20);
                        intake1150.setPower(IntakeInward);
                    } else if (timerRight.milliseconds() < stopIntake) {
                        intake1150.setPower(0);
                    } else if (timerRight.milliseconds() < shootSecond) { // same comment as above
                        finalIntakeRight.setPosition(0);
                    } else {
                        finalIntakeRight.setPosition(20);
                        rightFlywheel.setVelocity(0);
                        shootRight = false;
                        timerRight.reset();
                    }
                }
                break;
            case DRIVE_TO_COLLECT_1:

                intake1150.setPower(-1);

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 1.0) {
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

                leftFlywheel.setVelocity(-shootTicksPerSec);
                rightFlywheel.setVelocity(shootTicksPerSec);

                if (!follower.isBusy()) {
                    intake1150.setPower(0);
                    transition(State.SHOOT_2);
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

    /* ================= FLYWHEEL CONTROL ================= */
    private void runFlywheelAtTargetRPM() {
        // 6000 RPM motor → ~0.33 power for 2000 RPM
        double power = TARGET_RPM / 6000.0;
        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);
    }

    private void stopFlywheel() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }

    /* ================= RPM CALC ================= */
    private double getFlywheelRPM(DcMotor motor) {

        double currentTime = stateTimer.getElapsedTimeSeconds();
        int currentPosition = motor.getCurrentPosition();

        double deltaTime = currentTime - lastFlywheelTime;
        int deltaTicks = currentPosition - lastFlywheelPosition;

        if (deltaTime <= 0) return 0;

        double rpm = ((deltaTicks / TICKS_PER_REV) / deltaTime) * 60.0;

        lastFlywheelTime = currentTime;
        lastFlywheelPosition = currentPosition;

        return rpm;
    }

    /* ================= HELPERS ================= */
    private void transition(State next) {
        state = next;
        stateTimer.resetTimer();
    }
}
