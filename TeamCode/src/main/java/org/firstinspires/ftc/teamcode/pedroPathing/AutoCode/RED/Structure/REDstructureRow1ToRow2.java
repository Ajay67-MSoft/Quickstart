package org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.RED.Structure;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class REDstructureRow1ToRow2 extends OpMode {

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


    /* ================= STATES ================= */
    public enum State {
        statePathShootPreload,
        stateShootPreload,
        statePathToCollectRow2,
        statePathThatCollectsRow2,
        statePathPreparationMovementToMoveToShoot2,
        stateReturnFromRow2ToShoot,
        stateShootRow2,
        statePathToCollectRow1,
        statePathThatCollectsRow1,
        stateReturnFromRow1ToShoot,
        stateShootRow1,
        stateDriveToEnd,
        STATE_FINISHED
    }

    private State state;

    /* ================= POSES (FIXED RED MIRROR) ================= */
    private final Pose poseStart =
            new Pose(119, 131, Math.toRadians(37));
    private final Pose poseShootPreload =
            new Pose( 89, 100, Math.toRadians(38.5));
    private final Pose poseToCollect2 =
            new Pose(96, 64, Math.toRadians(0));
    private final Pose poseCollectsRow2 =
            new Pose(126, 60, Math.toRadians(0));
    private final Pose posePreparationPositionToMoveToShootPos =
            new Pose (100, 60, Math.toRadians(0));
    private final Pose poseShootRow2 =
            new Pose( 89, 100, Math.toRadians(38.5)); // increase x lower y to move farther from goal
    private final Pose poseToCollect1 =
            new Pose(99.6, 88, Math.toRadians(0));
    private final Pose poseCollectsRow1 =
            new Pose(135.0, 84, Math.toRadians(0));
    private final Pose poseShootRow1 =
            new Pose( 89, 100, Math.toRadians(38.5));
    private final Pose endPose =
            new Pose(99.6 + 16, 72, Math.toRadians(0));

    /* ================= PATHS ================= */
    private PathChain pathShootPreload;
    private PathChain pathToCollectRow2;
    private PathChain pathThatCollectsRow2;
    private PathChain pathPreparationMovementToMoveToShoot2;
    private PathChain pathReturnFromRow2ToShoot;
    private PathChain pathToCollectRow1;
    private PathChain pathThatCollectsRow1;
    private PathChain pathReturnFromRow1ToShoot;
    private PathChain pathDriveToEnd;

    /* ================= INIT ================= */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(poseStart);
        follower.setMaxPower(0.70);

        stateTimer = new Timer();

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        intake1150 = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        finalIntakeLeft = hardwareMap.get(CRServo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(CRServo.class, "finalIntakeServo");
        finalIntakeLeft.setDirection(CRServo.Direction.REVERSE);

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

        state = State.statePathShootPreload;
        stateTimer.resetTimer();
    }

    /* ================= PATH BUILDING ================= */
    private void buildPaths() {

        pathShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(poseStart, poseShootPreload))
                .setLinearHeadingInterpolation(
                        poseStart.getHeading(), poseShootPreload.getHeading())
                .build();

        pathToCollectRow1 = follower.pathBuilder()
                .addPath(new BezierLine(poseShootPreload, poseToCollect1))
                .setLinearHeadingInterpolation(poseShootPreload.getHeading(), poseToCollect1.getHeading())
                .build();

        pathThatCollectsRow1 = follower.pathBuilder()
                .addPath(new BezierLine(poseToCollect1, poseCollectsRow1))
                .setLinearHeadingInterpolation(
                        poseToCollect1.getHeading(), poseCollectsRow1.getHeading())
                .build();

        pathReturnFromRow1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(poseCollectsRow1, poseShootRow1))
                .setLinearHeadingInterpolation(
                        poseCollectsRow1.getHeading(), poseShootRow1.getHeading())
                .build();

        pathToCollectRow2 = follower.pathBuilder()
                .addPath(new BezierLine(poseShootRow1, poseToCollect2))
                .setLinearHeadingInterpolation(poseShootRow1.getHeading(), poseToCollect2.getHeading())
                .build();

        pathThatCollectsRow2 = follower.pathBuilder()
                .addPath(new BezierLine(poseToCollect2, poseCollectsRow2))
                .setLinearHeadingInterpolation(poseToCollect2.getHeading(), poseCollectsRow2.getHeading())
                .build();

        pathPreparationMovementToMoveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(poseCollectsRow2, posePreparationPositionToMoveToShootPos))
                .setLinearHeadingInterpolation(poseCollectsRow2.getHeading(), posePreparationPositionToMoveToShootPos.getHeading())
                .build();

        pathReturnFromRow2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(poseCollectsRow2, poseShootRow2))
                .setLinearHeadingInterpolation(poseCollectsRow2.getHeading(), poseShootRow2.getHeading())
                .build();

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(poseShootRow1, endPose))
                .setLinearHeadingInterpolation(
                        poseShootRow1.getHeading(), endPose.getHeading())
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

                leftFlywheel.setPower(-0.73);
                rightFlywheel.setPower(0.73);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 160) {
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

            case statePathToCollectRow2:
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(pathToCollectRow2, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);

                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.statePathThatCollectsRow2); // ------------------ TRANSITION STATES
                }
                break;

            case statePathThatCollectsRow2:
                if (!pathStarted) {
                    follower.setMaxPower(0.70);
                    follower.followPath(pathThatCollectsRow2, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);

                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.statePathPreparationMovementToMoveToShoot2); // ------------------ TRANSITION STATES
                }
                break;

            case statePathPreparationMovementToMoveToShoot2:
                if (!pathStarted) {
                    follower.setMaxPower(0.70);
                    follower.followPath(pathPreparationMovementToMoveToShoot2, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);

                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.stateReturnFromRow2ToShoot); // ------------------ TRANSITION STATES
                }
                break;

            case stateReturnFromRow2ToShoot:
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(pathReturnFromRow2ToShoot, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);

                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.stateShootRow2); // ------------------ TRANSITION STATES
                }
                break;

            case stateShootRow2:

                leftFlywheel.setPower(-0.73);
                rightFlywheel.setPower(0.73);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 160) {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    leftFlywheel.setPower(0.025);
                    rightFlywheel.setPower(-0.025);
                    intake1150.setPower(0);
                    transition(State.stateDriveToEnd); // ------------------ TRANSITION STATES
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

                leftFlywheel.setPower(-0.73);
                rightFlywheel.setPower(0.73);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 160) {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    intake1150.setPower(0);

                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    transition(State.statePathToCollectRow2); // ------------------ TRANSITION STATES
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

                intake1150.setPower(0);
                finalIntakeLeft.setPower(F_Intake_Hold);
                finalIntakeRight.setPower(F_Intake_Hold);
                break;
        }
    }

    /* ================= HELPERS ================= */
    private void transition(State next) {
        pathStarted = false;
        hasSetFinalIntakePowerToShoot = false;
        state = next;
        stateTimer.resetTimer();
    }
}
