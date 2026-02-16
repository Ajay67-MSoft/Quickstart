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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class FarEndAutoBLUE extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake1150;
    private Servo finalIntakeLeft;
    private Servo finalIntakeRight;

    // SHOOTING CONSTANTS
    private boolean shootLeft = false;
    private boolean shootRight = false;
    private int shootFirst = 500;
    private int prepareSecond = 1500;
    private int stopIntake = 3500;
    private int shootSecond = 4500;
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_SHOOT_RPM = 3225; // 2500 --> 2925
    private static final double RPM_TOLERANCE = 100;
    private final double shootTicksPerSec = TARGET_SHOOT_RPM * TICKS_PER_REV / 60.0;
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
        DRIVE_TO_COLLECT,
        FINISHED
    }
    private State state;

    /* ================= POSES ================= */
    // Mirrored from RED Far End Auto
    private final Pose startPose = new Pose(144 - 88, 8.2, Math.toRadians(90)); // mirror x
    private final Pose shootPose = new Pose(144 - 83.9802306425, 15.182866556836899, Math.toRadians(180 - 66));
    private final Pose endPose   = new Pose(42, 32, Math.toRadians(180));

    /* ================= PATHS ================= */
    private PathChain pathShoot1;
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

        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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

        pathDriveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {
        telemetry.addLine("Far End Auto BLUE (mirrored RED)");
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

                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE && !shootLeft) {
                    shootLeft = true;
                    timerLeft.reset();
                }
                if (Math.abs(rightRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE && !shootRight) {
                    shootRight = true;
                    timerRight.reset();
                }

                if (shootLeft) {
                    if (timerLeft.milliseconds() < shootFirst) {
                        finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    } else if (timerLeft.milliseconds() < prepareSecond) {
                        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                        intake1150.setPower(IntakeInward);
                    } else if (timerLeft.milliseconds() < stopIntake) {
                        intake1150.setPower(0);
                    } else if (timerLeft.milliseconds() < shootSecond) {
                        finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    } else {
                        finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                        leftFlywheel.setVelocity(0);
                        shootLeft = false;
                        timerLeft.reset();
                    }
                }

                if (shootRight) {
                    if (timerRight.milliseconds() < shootFirst) {
                        finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                    } else if (timerRight.milliseconds() < prepareSecond) {
                        finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                        intake1150.setPower(IntakeInward);
                    } else if (timerRight.milliseconds() < stopIntake) {
                        intake1150.setPower(0);
                    } else if (timerRight.milliseconds() < shootSecond) {
                        finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                    } else {
                        finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                        rightFlywheel.setVelocity(0);
                        shootRight = false;
                        timerRight.reset();
                    }
                }
                break;

            case FINISHED:
                follower.followPath(pathDriveToEnd, true);
                intake1150.setPower(0);
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
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
