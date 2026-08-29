package org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.Structure;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class BLUEstructureRow1ToRow2 extends OpMode {

    private boolean pathStarted = false;
    private boolean hasResetShootingTimer = false;

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
//    private final double shootTicksPerSec = SHOOT_RPM * TICKS_PER_REV / 60.0; commented out because not being used
    //    private static final double TARGET_RPM = 2000.0; commented out because not being used
    private static final double RPM_TOLERANCE = 180;
    private int IntakeInward = -1;
    private int IntakeOutward = 1;
    private int IntakeNoPower = 0;

    private static final double F_Intake_Shoot = 1.0;
    private static final double F_Intake_Backwards = -1.0;
    private static final double F_Intake_Hold = 0.0;
    private boolean shouldBeShooting = true;
    private ElapsedTime customTimer = new ElapsedTime();
    private boolean resetCustomTimer = false;

    /* ================= PEDRO ================= */
    private Follower follower;
    private Timer stateTimer;

    /* ================= RPM TRACKING ================= */
    private int lastFlywheelPosition = 0;
    private double lastFlywheelTime = 0.0;

    /* ================= STATES ================= */
    public enum State {
        STATE_DRIVE_AND_SHOOT_PRELOAD,
        STATE_INTAKE_AND_SCORE_ROW_1,
        STATE_INTAKE_AND_SCORE_ROW_2,
        STATE_FINISHED
    }

    private State state;

    /* ================= POSES ================= */
    private final Pose poseStart = new Pose(24.746955345060893, 128.60622462787552, Math.toRadians(143));
    private final Pose poseShootPreload = new Pose(55, 100, Math.toRadians(142)); // increase x lower y to move farther from goal
    private final Pose poseToCollect1 = new Pose(44.4, 85.5, Math.toRadians(180));
    private final Pose poseCollectsRow1 = new Pose(25, 81.5, Math.toRadians(180)); // kept ramming into ramp so changed x from 15 --> 25
    private final Pose poseShootRow1 = new Pose(55, 100, Math.toRadians(140)); // increase x lower y to move farther from goal
    private final Pose poseToCollect2 = new Pose(44.4, 62, Math.toRadians(180));
    private final Pose poseCollectsRow2 = new Pose(20, 57, Math.toRadians(180));
    private final Pose posePreparationPositionToMoveToShootPos = new Pose(44.4, 56, Math.toRadians(180));
    private final Pose poseShootRow2 = new Pose(55, 100, Math.toRadians(142)); // increase x lower y to move farther from goal
    private final Pose poseEnd = new Pose(24, 68, Math.toRadians(180));

    /* ================= PATHS ================= */
    private PathChain scorePreloadChain;
    private PathChain intakeAndScoreRow1Chain;
    private PathChain intakeAndScoreRow2Chain;
    //    private PathChain pathReturnFromRow1ToShoot;
//    private PathChain pathToCollectRow2;
//    private PathChain pathThatCollectsRow2;
//    private PathChain pathPreparationMovementToMoveToShoot2;
//    private PathChain pathReturnFromRow2ToShoot;
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

        PIDFCoefficients shooterPIDF =
                new PIDFCoefficients(0.35, 0.0, 0.002, 14);


        leftFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        rightFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        telemetry.addLine("15 POOPS ON EILEEN");

        buildPaths();

        state = State.STATE_DRIVE_AND_SHOOT_PRELOAD;
        stateTimer.resetTimer();
    }

    /* ================= PATH BUILDING ================= */
    private void buildPaths() {
        scorePreloadChain = follower.pathBuilder()
                .addPath(new BezierLine(poseStart, poseShootPreload))
                .setLinearHeadingInterpolation(poseStart.getHeading(), poseShootPreload.getHeading())
                .build();
        intakeAndScoreRow1Chain = follower.pathBuilder()
                .addPath(new BezierLine(poseShootPreload, poseToCollect1))
                .setLinearHeadingInterpolation(poseShootPreload.getHeading(), poseToCollect1.getHeading())

                .addPath(new BezierLine(poseToCollect1, poseCollectsRow1))
                .setLinearHeadingInterpolation(poseToCollect1.getHeading(), poseCollectsRow1.getHeading())

                .addPath(new BezierLine(poseCollectsRow1, poseShootRow1))
                .setLinearHeadingInterpolation(poseCollectsRow1.getHeading(), poseShootRow1.getHeading())
                .build();

        intakeAndScoreRow2Chain = follower.pathBuilder()
                .addPath(new BezierLine(poseShootRow1, poseToCollect2))
                .setLinearHeadingInterpolation(poseShootRow1.getHeading(), poseToCollect2.getHeading())

                .addPath(new BezierLine(poseToCollect2, poseCollectsRow2))
                .setLinearHeadingInterpolation(poseToCollect2.getHeading(), poseCollectsRow2.getHeading())

                .addPath(new BezierLine(poseCollectsRow2, posePreparationPositionToMoveToShootPos))
                .setLinearHeadingInterpolation(poseCollectsRow2.getHeading(), posePreparationPositionToMoveToShootPos.getHeading())

                .addPath(new BezierLine(posePreparationPositionToMoveToShootPos, poseShootRow2))
                .setLinearHeadingInterpolation(posePreparationPositionToMoveToShootPos.getHeading(), poseShootRow2.getHeading())

                .addPath(new BezierLine(poseShootRow2, poseEnd))
                .setLinearHeadingInterpolation(poseShootRow2.getHeading(), poseEnd.getHeading())
                .build();
    }

    /*
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
                    .addPath(new BezierLine(posePreparationPositionToMoveToShootPos, poseShootRow2))
                    .setLinearHeadingInterpolation(posePreparationPositionToMoveToShootPos.getHeading(), poseShootRow2.getHeading())
                    .build();

            pathDriveToEnd = follower.pathBuilder()
                    .addPath(new BezierLine(poseShootRow2, poseEnd))
                    .setLinearHeadingInterpolation(poseShootRow2.getHeading(), poseEnd.getHeading())
                    .build();
        }
     */
    private void spinUpFlywheelsHybrid(double currentLeftRPM) {
        // Threshold calculation: 400 RPM below your shooting target
//        double thresholdRPM = 2200;

        if (Math.abs(currentLeftRPM) < 2200) {
            // 1. BANG-BANG
            // put max flywheel power lol
            leftFlywheel.setPower(-1.0);
            rightFlywheel.setPower(1.0);
        } else {
            // PIDF
            // Speed is within 400 RPM: switch back to pidf
            leftFlywheel.setVelocity(-1167); // 1633
            rightFlywheel.setVelocity(1167); // 1633
        }
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {
        follower.update();
        updateStateMachine();
    }

    /* ================= STATE MACHINE ================= */
    private void updateStateMachine() {

        // telemetry -=-=-=-=-=-=-=-=-
        double leftRPM = leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        // double rightRPM = rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        telemetry.addData("Current Flywheel RPM:", "%.1f", Math.abs(leftRPM));
        // telemetry.addData("Flywheel RPM Right:", "%.1f", rightRPM);
        telemetry.addData("Target RPM", TARGET_SHOOT_RPM);
        // telemetry.addData("timer: left", timerLeft.milliseconds());
        // telemetry.addData("timer right:", timerRight.milliseconds());
        telemetry.update();
        // telemetry -=-=-=-=-=-=-=-=-


        double currentSegment = follower.getCurrentPathNumber();
        switch (state) {
            case STATE_DRIVE_AND_SHOOT_PRELOAD:
                if (shouldBeShooting) {
                    spinUpFlywheelsHybrid(leftRPM);
                }
                else {
                    leftFlywheel.setPower(0.025);
                    rightFlywheel.setPower(-0.025);
                }

                if (!pathStarted) {
                    follower.setMaxPower(1.0);
                    follower.followPath(scorePreloadChain, true);
                    pathStarted = true;
                }

                // Since arrived at the end of the preload path, we'll shoot the artifacts:
                if (!follower.isBusy()) {
                    if (!resetCustomTimer) {
                        customTimer.reset();
                        resetCustomTimer = true;
                    }

                    if (customTimer.milliseconds() >= 5000) {
                        finalIntakeRight.setPower(F_Intake_Hold);
                        finalIntakeLeft.setPower(F_Intake_Hold);
                        intake1150.setPower(0);
                        // timer
                        resetCustomTimer = false;
                        // stop flywheels basically
                        shouldBeShooting = false;
                        transition(State.STATE_INTAKE_AND_SCORE_ROW_1); // ------------------ TRANSITION STATES
                    } else if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE) {
                        finalIntakeRight.setPower(F_Intake_Shoot);
                        finalIntakeLeft.setPower(F_Intake_Shoot);
                        intake1150.setPower(IntakeInward);
                    } else {
                        finalIntakeRight.setPower(F_Intake_Hold);
                        finalIntakeLeft.setPower(F_Intake_Hold);
                        intake1150.setPower(0);
                    }
                }
                break;
            case STATE_INTAKE_AND_SCORE_ROW_1:
                if (shouldBeShooting) {
                    spinUpFlywheelsHybrid(leftRPM);
                }
                else {
                    leftFlywheel.setPower(0.025);
                    rightFlywheel.setPower(-0.025);
                }

                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(intakeAndScoreRow1Chain, true);
                    pathStarted = true;
                }

                // ----- controlling subsystems dynamically using path number -----
                // Path 0: poseShootPreload -> poseToCollect1
                // Path 1: poseToCollect1 -> poseCollectsRow1
                // Path 2: poseCollectsRow1 -> poseShootRow1

                if (currentSegment == 0 || currentSegment == 1) {
                    // start intake, spin final intake backwards to prevent popping
                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);
                    shouldBeShooting = false;
                } else if (currentSegment == 2) {
                    // intakes off, flywheels on
                    intake1150.setPower(0);
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    shouldBeShooting = true;
                }

                // shoot artifacts
                if (!follower.isBusy()) {
                    shouldBeShooting = true;

                    if (!resetCustomTimer) {
                        customTimer.reset();
                        resetCustomTimer = true;
                    }

                    if (customTimer.milliseconds() >= 5000) { // times up, move to next pathchain
                        finalIntakeRight.setPower(F_Intake_Hold);
                        finalIntakeLeft.setPower(F_Intake_Hold);
                        intake1150.setPower(0);
                        // timer
                        resetCustomTimer = false;
                        // stop flywheels basically
                        shouldBeShooting = false;
                        transition(State.STATE_INTAKE_AND_SCORE_ROW_2); // ------------------ TRANSITION STATES
                    } else if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE) {
                        finalIntakeRight.setPower(F_Intake_Shoot);
                        finalIntakeLeft.setPower(F_Intake_Shoot);
                        intake1150.setPower(IntakeInward);
                    } else {
                        finalIntakeRight.setPower(F_Intake_Hold);
                        finalIntakeLeft.setPower(F_Intake_Hold);
                        intake1150.setPower(0);
                    }
                }
                break;

            case STATE_INTAKE_AND_SCORE_ROW_2:
                if (shouldBeShooting) {
                    spinUpFlywheelsHybrid(leftRPM);
                }
                else {
                    leftFlywheel.setPower(0.025);
                    rightFlywheel.setPower(-0.025);
                }

                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(intakeAndScoreRow2Chain, true);
                    pathStarted = true;
                }
                // Path 0: poseShootRow1 -> poseToCollect2
                // Path 1: poseToCollect2 -> poseCollectsRow2
                // Path 2: poseCollectsRow2 ->
                // posePreparationPositionToMoveToShootPos
                // Path 3: posePreparationPositionToMoveToShootPos ->
                // poseShootRow2
                // Path 4: poseShootRow2 -> poseEnd
                if (currentSegment == 0 || currentSegment == 1) {
                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);
                } else if (currentSegment == 2 || currentSegment == 3) {
                    intake1150.setPower(0);
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    shouldBeShooting = true;
                } else if (currentSegment == 4) {
                    intake1150.setPower(0);
                    shouldBeShooting = false;
                }


                if (!follower.isBusy() && currentSegment == 3) {
                    shouldBeShooting = true;

                    if (!resetCustomTimer) {
                        customTimer.reset();
                        resetCustomTimer = true;
                    }

                    if (customTimer.milliseconds() >= 5000) {
                        finalIntakeRight.setPower(F_Intake_Hold);
                        finalIntakeLeft.setPower(F_Intake_Hold);
                        intake1150.setPower(0);
                        // timer
                        resetCustomTimer = false;
                        // stop flywheels basically
                        shouldBeShooting = false;
                        transition(State.STATE_FINISHED);
                    } else if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE) {
                        finalIntakeRight.setPower(F_Intake_Shoot);
                        finalIntakeLeft.setPower(F_Intake_Shoot);
                        intake1150.setPower(IntakeInward);
                    } else {
                        finalIntakeRight.setPower(F_Intake_Hold);
                        finalIntakeLeft.setPower(F_Intake_Hold);
                        intake1150.setPower(0);
                    }
                }
                break;

            case STATE_FINISHED:
                follower.update();
                leftFlywheel.setPower(0);
                rightFlywheel.setPower(0);
                intake1150.setPower(0);
                finalIntakeLeft.setPower(F_Intake_Hold);
                finalIntakeRight.setPower(F_Intake_Hold);

                if (!follower.isBusy()) {
                    requestOpModeStop();
                }

                break;
        }
    }

    /* ================= HELPERS ================= */
    private void transition(State next) {
        pathStarted = false;
        state = next;
        stateTimer.resetTimer();
    }
}
