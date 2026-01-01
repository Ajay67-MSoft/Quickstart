package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
GOALS WITH THIS COMMIT
1. fix how fast flywheels spin at max battery voltage
 */

@TeleOp
public class FarEndAutoBLUE extends OpMode {

    private boolean pathStarted = false;

    /* ================= HARDWARE ================= */

    private DcMotor leftFlywheel;
    private DcMotor rightFlywheel;
    private DcMotor intake1150;
    private Servo finalIntakeLeft;
    private Servo finalIntakeRight;

    private double leftFlywheelPower = -0.58-.22; // orignal was 0.58, went to -0.38 to account for max voltage battery
    private double rightFlywheelPower = 0.5-.12; // original was 0.5

    private double flywheelRampUpDurationSeconds = 3.0;

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

    private final Pose startPose = new Pose(56, 8.2, Math.toRadians(90));
    private final Pose shootPose = new Pose(60.019769357495875, 15.182866556836899, Math.toRadians(122.5));
    private final Pose endPose = new Pose(56, 8.2, Math.toRadians(90));

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

        leftFlywheel  = hardwareMap.get(DcMotor.class, "6000 RPM motor");
        rightFlywheel = hardwareMap.get(DcMotor.class, "6000 RPM motor flywheel right");
        intake1150    = hardwareMap.get(DcMotor.class, "1150 RPM intake");



        intake1150.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        telemetry.addLine("1 POOPS ON ANDY"); // ------------ VERY IMPORTANT VERSION NUMBER LINE -----------
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

                // start flywheels
//                rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftFlywheel.setPower(leftFlywheelPower);
                rightFlywheel.setPower(rightFlywheelPower);

                // shoot first two balls
                // add 3.7 sec to account for farther distance
                if (t > 6.7) { // original: 3 sec
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                // reset final intake servo

                if (t > 7.7) { // original: 4 sec
                    finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
                    finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                }

                // start intake servo to move third ball

                if (t > 8.7) { // original: 5 sec
                    intake1150.setPower(-1);
                }

                // because flywheels are still running,
                // use final intake servo to shoot third ball

                if (t > 9.7) { // original: 6 sec
                    finalIntakeLeft.setPosition(SERVO_FEED_POSITION);
                    finalIntakeRight.setPosition(SERVO_FEED_POSITION);
                }

                // stop all motors because we have no balls

                if (t > 10.7) { // original: 7 sec
                    // stop flywheels
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                    // reset servo positions
//                    finalIntakeLeft.setPosition(SERVO_STOP_POSITION);
//                    finalIntakeRight.setPosition(SERVO_STOP_POSITION);
                    // stop first intake servo
                    intake1150.setPower(0);
                    transition(State.DRIVE_TO_COLLECT);
                }
                break;
            case FINISHED:
                follower.followPath(pathDriveToEnd, true);
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
