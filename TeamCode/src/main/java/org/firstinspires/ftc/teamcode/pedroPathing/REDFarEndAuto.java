package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
GOALS WITH THIS COMMIT
1. create red side pos by doing (144 - current x) and (current y)
2. create red side angle by doing (180 - current angle)
 */

@Autonomous
public class REDFarEndAuto extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake1150;
    private CRServo finalIntakeLeft;
    private CRServo finalIntakeRight;

    // limelight ty: 16.5 - closest possible (in front of purple line) (2400-2500 RPM)
    private double SHOOT_RPM = 3000; // 2650 (gap 150 from target_shoot_rpm) --> +425 --> 3075
    private double TARGET_SHOOT_RPM = 3000; // 2500 --> 2925
    private static final double TICKS_PER_REV = 28.0;
    private final double shootTicksPerSec = TARGET_SHOOT_RPM * TICKS_PER_REV / 60.0;
    private static final double RPM_TOLERANCE = 125;
    private int IntakeInward = -1;
    private int IntakeOutward = 1;
    private int IntakeNoPower = 0;

    private static final double F_Intake_Shoot = 1.0;
    private static final double F_Intake_Backwards = -1.0;
    private static final double F_Intake_Hold = 0.0;

    private ElapsedTime timerLeft = new ElapsedTime();
    private ElapsedTime timerRight = new ElapsedTime();

    /* ================= PEDRO ================= */

    private Follower follower;
    private Timer stateTimer;

    /* ================= STATES ================= */

    public enum State {
        DRIVE_TO_SHOOT_1,
        SHOOT_1,
        DRIVE_TO_COLLECT,
        FINISHED
    }

    private State state;

    /* ================= POSES ================= */

    private final Pose startPose = new Pose(88, 8.2, Math.toRadians(90));
    private final Pose shootPose = new Pose(83.9802306425, 15.182866556836899, Math.toRadians(66));
    private final Pose endPose = new Pose(100, 32, Math.toRadians(0));

    /* ================= PATHS ================= */

    private PathChain pathShoot1;
    private PathChain pathDriveToEnd;

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



        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake1150.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1150.setPower(0);

        finalIntakeLeft  = hardwareMap.get(CRServo.class, "FinalIntakeLeftDS");
        finalIntakeRight = hardwareMap.get(CRServo.class, "finalIntakeServo");

        finalIntakeLeft.setDirection(CRServo.Direction.REVERSE);

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

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {
        telemetry.addLine("1 RED POOP IN THE BACK ;-; ;-; ;-; ;-; ;-;"); // ------------ VERY IMPORTANT VERSION NUMBER LINE -----------
        follower.update();
        updateStateMachine();
    }

    /* ================= STATE MACHINE ================= */

    private void updateStateMachine() {

        double leftRPM = leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        double rightRPM = rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV;

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
                leftFlywheel.setVelocity(-shootTicksPerSec - 11.6);
                rightFlywheel.setVelocity(shootTicksPerSec);

                if (stateTimer.getElapsedTimeSeconds() >= 15) {
                    transition(State.FINISHED);
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
            case FINISHED:
                follower.followPath(pathDriveToEnd, true);

                leftFlywheel.setPower(0);
                rightFlywheel.setPower(0);

                intake1150.setPower(0);

                finalIntakeLeft.setPower(0);
                finalIntakeRight.setPower(0);
                break;
        }
    }

    /* ================= HELPERS ================= */

    private void transition(State next) {
        state = next;
        stateTimer.resetTimer();
    }
}
