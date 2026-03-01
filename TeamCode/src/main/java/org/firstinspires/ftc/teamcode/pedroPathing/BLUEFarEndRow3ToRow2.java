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
import com.qualcomm.robotcore.hardware.CRServo;

/*
GOALS WITH THIS COMMIT
1. create red side pos by doing (144 - current x) and (current y)
2. create red side angle by doing (180 - current angle)
 */

@Autonomous
public class BLUEFarEndRow3ToRow2 extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake1150;
    private CRServo finalIntakeLeft;
    private CRServo finalIntakeRight;

    // limelight ty: 16.5 - closest possible (in front of purple line) (2400-2500 RPM)
//    private double SHOOT_RPM = 3000; // 2650 (gap 150 from target_shoot_rpm) --> +425 --> 3075
    private double TARGET_SHOOT_RPM = 3000; // 2500 --> 2925
    private static final double TICKS_PER_REV = 28.0;
    //    private final double shootTicksPerSec = TARGET_SHOOT_RPM * TICKS_PER_REV / 60.0;
    private static final double RPM_TOLERANCE = 125;
    private int IntakeInward = -1;
    private int IntakeOutward = 1;
    private int IntakeNoPower = 0;

    private static final double F_Intake_Shoot = 1.0;
    private static final double F_Intake_Backwards = -1.0;
    private static final double F_Intake_Hold = 0.0;
    private boolean hasSetFinalIntakePowerToShoot = false;
//    private ElapsedTime timerLeft = new ElapsedTime();
//    private ElapsedTime timerRight = new ElapsedTime();

    /* ================= PEDRO ================= */

    private Follower follower;
    private Timer stateTimer;

    /* ================= STATES ================= */

    public enum State {
        statePathShootPreload,
        stateShootPreload,
        statePathToCollectRow2,
        statePathThatCollectsRow2,
        stateReturnFromRow2ToShoot,
        stateShootRow2,
        statePathToCollectRow3,
        statePathThatCollectsRow3,
        stateReturnFromRow3ToShoot,
        stateShootRow3,
        stateDriveToEnd,
        STATE_FINISHED
    }

    private State state;

    /* ================= POSES ================= */
    /*
    good poses
    private final Pose poseStart = new Pose(56, 8.2, Math.toRadians(90));
    private final Pose poseShootPreload = new Pose(60, 15, Math.toRadians(114)); // increase x lower y to move farther from goal
    private final Pose poseShootRow2 = new Pose(60, 15, Math.toRadians(114)); // increase x lower y to move farther from goal
    private final Pose poseShootRow3 = new Pose(60, 15, Math.toRadians(114)); // increase x lower y to move farther from goal

    switch poses

    private final Pose poseToCollect2 = new Pose(102, 61, Math.toRadians(180));

    private final Pose poseCollectsRow2 = new Pose(128, 57, Math.toRadians(180));

    private final Pose poseToCollect3 = new Pose(107, 38, Math.toRadians(180));
    private final Pose poseCollectsRow3 = new Pose(132, 34, Math.toRadians(180));

    private final Pose poseEnd = new Pose(107, 13, Math.toRadians(0));
     */
    private final Pose poseStart = new Pose(56, 8.2, Math.toRadians(90));
    private final Pose poseShootPreload = new Pose(60, 15, Math.toRadians(114)); // increase x lower y to move farther from goal
    private final Pose poseToCollect2 = new Pose(44.4, 60, Math.toRadians(180));
    private final Pose poseCollectsRow2 = new Pose(15, 56, Math.toRadians(180));
    private final Pose poseShootRow2 = new Pose(60, 15, Math.toRadians(117)); // increase x lower y to move farther from goal
    private final Pose poseToCollect3 = new Pose(43, 39, Math.toRadians(180));
    private final Pose poseCollectsRow3 = new Pose(15, 35, Math.toRadians(180));
    private final Pose poseShootRow3 = new Pose(60, 15, Math.toRadians(117)); // increase x lower y to move farther from goal
    private final Pose poseEnd = new Pose(38, 25, Math.toRadians(180));

    /* ================= PATHS ================= */

    private PathChain pathShootPreload;
    private PathChain pathToCollectRow2;
    private PathChain pathThatCollectsRow2;
    private PathChain pathReturnFromRow2ToShoot;
    private PathChain pathToCollectRow3;
    private PathChain pathThatCollectsRow3;
    private PathChain pathReturnFromRow3ToShoot;
    private PathChain pathDriveToEnd;

    /* ================= INIT ================= */

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(poseStart);
        follower.setMaxPower(0.70);

        stateTimer = new Timer();

        leftFlywheel  = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        intake1150    = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1150.setPower(0);

        finalIntakeLeft  = hardwareMap.get(CRServo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(CRServo.class, "finalIntakeServo");

        finalIntakeLeft.setDirection(CRServo.Direction.REVERSE);

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

        pathToCollectRow2 = follower.pathBuilder()
                .addPath(new BezierLine(poseShootRow3, poseToCollect2))
                .setLinearHeadingInterpolation(poseShootRow3.getHeading(), poseToCollect2.getHeading())
                .build();

        pathThatCollectsRow2 = follower.pathBuilder()
                .addPath(new BezierLine(poseToCollect2, poseCollectsRow2))
                .setLinearHeadingInterpolation(poseToCollect2.getHeading(), poseCollectsRow2.getHeading())
                .build();

        pathReturnFromRow2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(poseCollectsRow2, poseShootRow2))
                .setLinearHeadingInterpolation(poseCollectsRow2.getHeading(), poseShootRow2.getHeading())
                .build();

        pathToCollectRow3 = follower.pathBuilder()
                .addPath(new BezierLine(poseShootPreload, poseToCollect3))
                .setLinearHeadingInterpolation(poseShootPreload.getHeading(), poseToCollect3.getHeading())
                .build();

        pathThatCollectsRow3 = follower.pathBuilder()
                .addPath(new BezierLine(poseToCollect3, poseCollectsRow3))
                .setLinearHeadingInterpolation(poseToCollect3.getHeading(), poseCollectsRow3.getHeading())
                .build();

        pathReturnFromRow3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(poseCollectsRow3, poseShootRow3))
                .setLinearHeadingInterpolation(poseCollectsRow3.getHeading(), poseShootRow3.getHeading())
                .build();

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(poseShootRow2, poseEnd))
                .setLinearHeadingInterpolation(poseShootRow2.getHeading(), poseEnd.getHeading())
                .build();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {
//        telemetry.addLine("1 RED POOP IN THE BACK ;-; ;-; ;-; ;-; ;-;"); // ------------ VERY IMPORTANT VERSION NUMBER LINE -----------
        follower.update();
        updateStateMachine();
    }

    /* ================= STATE MACHINE ================= */

    private void updateStateMachine() {

        double leftRPM = leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        double rightRPM = rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;

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

                leftFlywheel.setPower(-0.7);
                rightFlywheel.setPower(0.7);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 500) {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    leftFlywheel.setPower(0.025);
                    rightFlywheel.setPower(-0.025);
                    intake1150.setPower(0);
                    transition(State.statePathToCollectRow3); // ------------------ TRANSITION STATES
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
                    pathStarted = true;
                }

                intake1150.setPower(IntakeInward);
                finalIntakeRight.setPower(F_Intake_Backwards);
                finalIntakeLeft.setPower(F_Intake_Backwards);

                if (!follower.isBusy()) {
                    transition(State.statePathThatCollectsRow2); // ------------------ TRANSITION STATES
                }
                break;

            case statePathThatCollectsRow2:

                if (!pathStarted) {
                    follower.setMaxPower(0.70);
                    follower.followPath(pathThatCollectsRow2, true); // ------------------ FOLLOWER
                    pathStarted = true;
                }

                intake1150.setPower(IntakeInward);
                finalIntakeRight.setPower(F_Intake_Backwards);
                finalIntakeLeft.setPower(F_Intake_Backwards);


                if (!follower.isBusy()) {
                    transition(State.stateReturnFromRow2ToShoot); // ------------------ TRANSITION STATES
                }
                break;

            case stateReturnFromRow2ToShoot:

                if (!pathStarted) {
                    follower.setMaxPower(0.8);
                    follower.followPath(pathReturnFromRow2ToShoot, true); // ------------------ FOLLOWER
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    intake1150.setPower(0);
                    transition(State.stateShootRow2); // ------------------ TRANSITION STATES
                }
                break;

            case stateShootRow2:

                leftFlywheel.setPower(-0.7);
                rightFlywheel.setPower(0.7);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 500) {
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

            case statePathToCollectRow3:
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(pathToCollectRow3, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);

                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.statePathThatCollectsRow3); // ------------------ TRANSITION STATES
                }
                break;

            case statePathThatCollectsRow3:
                if (!pathStarted) {
                    follower.setMaxPower(0.70);
                    follower.followPath(pathThatCollectsRow3, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);

                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.stateReturnFromRow3ToShoot); // ------------------ TRANSITION STATES
                }
                break;

            case stateReturnFromRow3ToShoot:
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(pathReturnFromRow3ToShoot, true); // ------------------ FOLLOWER
                    leftFlywheel.setPower(0.1);
                    rightFlywheel.setPower(-0.1);

                    intake1150.setPower(IntakeInward);
                    finalIntakeRight.setPower(F_Intake_Backwards);
                    finalIntakeLeft.setPower(F_Intake_Backwards);

                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    transition(State.stateShootRow3); // ------------------ TRANSITION STATES
                }
                break;

            case stateShootRow3:

                leftFlywheel.setPower(-0.7);
                rightFlywheel.setPower(0.7);

                if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 500) {
                    finalIntakeRight.setPower(F_Intake_Hold);
                    finalIntakeLeft.setPower(F_Intake_Hold);
                    leftFlywheel.setPower(0.025);
                    rightFlywheel.setPower(-0.025);
                    intake1150.setPower(0);
                    transition(State.statePathToCollectRow2); // ------------------ TRANSITION STATES
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

    /* ================= HELPERS ================= */

    private void transition(State next) {
        pathStarted = false;
        hasSetFinalIntakePowerToShoot = false;
        state = next;
        stateTimer.resetTimer();
    }
}
