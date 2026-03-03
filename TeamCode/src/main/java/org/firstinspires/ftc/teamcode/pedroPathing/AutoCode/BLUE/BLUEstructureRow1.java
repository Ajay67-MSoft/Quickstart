package org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 GOALS WITH THIS COMMIT
 1. yes
 2. no
 3. maybe so
 */


public class BLUEstructureRow1 extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake1150;

    private CRServo finalIntakeLeft;
    private CRServo finalIntakeRight;



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
    //    private static final double TARGET_RPM = 2000.0; commented out because not being used
    private static final double RPM_TOLERANCE = 180;
    private int IntakeInward = -1;
    private int IntakeOutward = 1;
    private int IntakeNoPower = 0;

    private static final double F_Intake_Shoot = 1.0;
    private static final double F_Intake_Backwards = -1.0;
    private static final double F_Intake_Hold = 0.0;

    private boolean hasSetFinalIntakePowerToShoot = false;

    /* ================= PEDRO ================= */
    private Follower follower;
    private Timer stateTimer;

    /* ================= RPM TRACKING ================= */
    private int lastFlywheelPosition = 0;
    private double lastFlywheelTime = 0.0;

    /* ================= STATES ================= */
    public enum State {
        statePathShootPreload,
        stateShootPreload,
        statePathToCollectRow1,
        statePathThatCollectsRow1,
        stateReturnFromRow1ToShoot,
        stateShootRow1,
        stateDriveToEnd,
        STATE_FINISHED
        /*
        private PathChain pathShoot1; // startPose --> shootPose, to call state use DRIVE_TO_SHOOT_1,
    private PathChain pathToCollectRow1; // shootPose --> collect1, to call state use DRIVE_TO_COLLECT_1,
    private PathChain pathThatCollectsRow1; // collect1 --> collect2, to call state use COLLECT_1
    private PathChain pathReturnFromRow1ToShoot;
    private PathChain pathDriveToEnd;
         */
    }

    private State state;

    /* ================= POSES ================= */
    private final Pose poseStart = new Pose(24.746955345060893, 128.60622462787552, Math.toRadians(143));
    private final Pose poseShootPreload = new Pose(55, 100, Math.toRadians(142)); // increase x lower y to move farther from goal
    private final Pose poseToCollect1 = new Pose(44.4, 84, Math.toRadians(180));
    private final Pose poseCollectsRow1 = new Pose(15, 80, Math.toRadians(180));
    private final Pose poseShootRow1 = new Pose(55, 100, Math.toRadians(140)); // increase x lower y to move farther from goal
    private final Pose poseEnd = new Pose(24, 68, Math.toRadians(180));

    /* ================= PATHS ================= */
    private PathChain pathShootPreload;
    private PathChain pathToCollectRow1;
    private PathChain pathThatCollectsRow1;
    private PathChain pathReturnFromRow1ToShoot;
    private PathChain pathDriveToEnd;

    /* ================= INIT ================= */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(poseStart);
        follower.setMaxPower(0.85); // set power speed of the follower auto pedropathing

        stateTimer = new Timer();

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        intake1150 = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1150.setPower(0);

        finalIntakeLeft = hardwareMap.get(CRServo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(CRServo.class, "finalIntakeServo");
        finalIntakeLeft.setDirection(CRServo.Direction.REVERSE);

        leftFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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

        state = State.statePathShootPreload;
        stateTimer.resetTimer();
    }

    /* ================= PATH BUILDING ================= */
    private void buildPaths() {

        pathShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(poseStart, poseShootPreload))
                .setLinearHeadingInterpolation(poseStart.getHeading(), poseShootPreload.getHeading())
                .build();

        pathToCollectRow1 = follower.pathBuilder()
                .addPath(new BezierLine(poseShootPreload, poseToCollect1))
                .setLinearHeadingInterpolation(poseShootPreload.getHeading(), poseToCollect1.getHeading())
                .build();

        pathThatCollectsRow1 = follower.pathBuilder()
                .addPath(new BezierLine(poseToCollect1, poseCollectsRow1))
                .setLinearHeadingInterpolation(poseToCollect1.getHeading(), poseCollectsRow1.getHeading())
                .build();

        pathReturnFromRow1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(poseCollectsRow1, poseShootRow1))
                .setLinearHeadingInterpolation(poseCollectsRow1.getHeading(), poseShootRow1.getHeading())
                .build();

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(poseShootRow1, poseEnd))
                .setLinearHeadingInterpolation(poseShootRow1.getHeading(), poseEnd.getHeading())
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
//        double rightRPM = rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;
//        commented out cuz its not being used

//        telemetry.addData("Flywheel RPM Left:", "%.1f", leftRPM);
//        telemetry.addData("Flywheel RPM Right:", "%.1f", rightRPM);
//        telemetry.addData("Target RPM", TARGET_RPM);
//        telemetry.addData("timer: left", timerLeft.milliseconds());
//        telemetry.addData("timer right:", timerRight.milliseconds());
//        telemetry.update();
//        telemetry commented out cuz lets be honest - was it really doing anything



        switch (state) {

            case statePathShootPreload:
                follower.setMaxPower(0.85);

                if (!pathStarted) { // starts all of the paths, but required to put inside all of
                    leftFlywheel.setPower(0.15);
                    rightFlywheel.setPower(-0.15);
                    follower.followPath(pathShootPreload, true); // ------------------ FOLLOWER
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    transition(State.stateShootPreload); // ------------------ TRANSITION STATES
                }
                break;

            case stateShootPreload:

                leftFlywheel.setPower(-0.5);
                rightFlywheel.setPower(0.5);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 200) {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    leftFlywheel.setPower(0.025);
                    rightFlywheel.setPower(-0.025);
                    intake1150.setPower(0);
                    transition(State.statePathToCollectRow1); // ------------------ TRANSITION STATES
                }
                else if (hasSetFinalIntakePowerToShoot) {
                    // KEEP FEEDING, regardless of RPM dips
                    finalIntakeRight.setPower(F_Intake_Shoot);
                    finalIntakeLeft.setPower(F_Intake_Shoot);
                    intake1150.setPower(IntakeInward);
                }
                else if (!hasSetFinalIntakePowerToShoot &&
                        Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE) {

                    finalIntakeRight.setPower(F_Intake_Shoot);
                    finalIntakeLeft.setPower(F_Intake_Shoot);
                    intake1150.setPower(IntakeInward);
                    hasSetFinalIntakePowerToShoot = true;
                }
                else {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    intake1150.setPower(0);
                }
                // new shootLeft() and shootRight() function

                break;

            case statePathToCollectRow1:

                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(pathToCollectRow1, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);
                    pathStarted = true;
                }

                intake1150.setPower(IntakeInward);
                finalIntakeRight.setPower(F_Intake_Backwards);
                finalIntakeLeft.setPower(F_Intake_Backwards);

                if (!follower.isBusy()) {
                    transition(State.statePathThatCollectsRow1); // ------------------ TRANSITION STATES
                }
                break;

            case statePathThatCollectsRow1:

                if (!pathStarted) {
                    follower.setMaxPower(0.70);
                    follower.followPath(pathThatCollectsRow1, true); // ------------------ FOLLOWER
                    pathStarted = true;
                }

                intake1150.setPower(IntakeInward);
                finalIntakeRight.setPower(F_Intake_Backwards);
                finalIntakeLeft.setPower(F_Intake_Backwards);


                if (!follower.isBusy()) {
                    transition(State.stateReturnFromRow1ToShoot); // ------------------ TRANSITION STATES
                }
                break;

            case stateReturnFromRow1ToShoot:

                if (!pathStarted) {
                    follower.setMaxPower(0.8);
                    follower.followPath(pathReturnFromRow1ToShoot, true); // ------------------ FOLLOWER
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    intake1150.setPower(0);
                    transition(State.stateShootRow1); // ------------------ TRANSITION STATES
                }
                break;

            case stateShootRow1:

                leftFlywheel.setPower(-0.5);
                rightFlywheel.setPower(0.5);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 225) {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    intake1150.setPower(0);

                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    transition(State.stateDriveToEnd); // ------------------ TRANSITION STATES
                }
                else if (hasSetFinalIntakePowerToShoot) {
                    // KEEP FEEDING, regardless of RPM dips
                    finalIntakeRight.setPower(F_Intake_Shoot);
                    finalIntakeLeft.setPower(F_Intake_Shoot);
                    intake1150.setPower(IntakeInward);
                }
                // new shootLeft() and shootRight() function
                else if (!hasSetFinalIntakePowerToShoot &&
                        Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE) {

                    finalIntakeRight.setPower(F_Intake_Shoot);
                    finalIntakeLeft.setPower(F_Intake_Shoot);
                    intake1150.setPower(IntakeInward);
                    hasSetFinalIntakePowerToShoot = true;
                }
                else {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    intake1150.setPower(0);
                }
                break;

            case stateDriveToEnd:
                if (!pathStarted) {
                    follower.setMaxPower(0.80);
                    follower.followPath(pathDriveToEnd, true); // ------------------ FOLLOWER
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.STATE_FINISHED); // ------------------ TRANSITION STATES
                }
                break;

            case STATE_FINISHED:
                leftFlywheel.setPower(0);
                rightFlywheel.setPower(0);

                intake1150.setPower(0);
                finalIntakeLeft.setPower(F_Intake_Hold);
                finalIntakeRight.setPower(F_Intake_Hold);
                break;
        }
    }


//    /* ================= RPM CALC ================= */
//    private double getFlywheelRPM(DcMotor motor) {
//
//        double currentTime = stateTimer.getElapsedTimeSeconds();
//        int currentPosition = motor.getCurrentPosition();
//
//        double deltaTime = currentTime - lastFlywheelTime;
//        int deltaTicks = currentPosition - lastFlywheelPosition;
//
//        if (deltaTime <= 0) return 0;
//
//        double rpm = ((deltaTicks / TICKS_PER_REV) / deltaTime) * 60.0;
//
//        lastFlywheelTime = currentTime;
//        lastFlywheelPosition = currentPosition;
//
//        return rpm;
//    }

    /* ================= HELPERS ================= */
    private void transition(State next) {
        pathStarted = false;
        hasSetFinalIntakePowerToShoot = false;
        state = next;
        stateTimer.resetTimer();
    }
}
