package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

// imports to make this work: double distance = detection.ftcPose.z;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "AutoAimingTest")
public class AutoAimingTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;


    private Servo FinalIntakeLeftDS;
    private Servo finalIntakeServo;
    private DcMotor frontLeftWheelDS;
    private DcMotor backLeftWheelDS;
    private DcMotor frontRightWheelDS;
    private DcMotor backRightWheelDS;
    private DcMotor _1150RPMintake;
    private DcMotorEx _6000RPMmotor;
    private DcMotorEx _6000RPMmotorflywheelright;
    private Limelight3A limelight;

    private double x, y, rx;
    // --- variables ---



    private double targetLeftFlywheelVelocity = -1500;
    private double targetRightFlywheelVelocity = 1500;



    // --- end ---
    private boolean shoot = false; //double t = stateTimer.getElapsedTimeSeconds();
    private ElapsedTime timer = new ElapsedTime();

//    private int shootTime = 400;

    private int shootGap = 2000;
    private int shootFirst = 500;
    private int prepareSecond = 1000;
    private int shootSecond = 1500;
    private boolean isAlignedWithTarget;
//    private double angleOffset;
    // add this later when we're able to auto shoot
    private double distance;


    /**
     * idk man figure it out
     */
    @Override
    public void runOpMode() {

        FinalIntakeLeftDS = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeServo = hardwareMap.get(Servo.class, "finalIntakeServo");
        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");
        _1150RPMintake = hardwareMap.get(DcMotor.class, "1150 RPM intake");
        _6000RPMmotor = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        _6000RPMmotorflywheelright = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        frontRightWheelDS.setDirection(DcMotorEx.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotorEx.Direction.REVERSE);

        // Put initialization blocks here.
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        FinalIntakeLeftDS.setPosition(20);
        finalIntakeServo.setDirection(Servo.Direction.REVERSE);
        finalIntakeServo.setPosition(20);
        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(10.0, 3.0, 0.0, 12.0);
        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);






        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                y = gamepad1.left_stick_y;
                x = -gamepad1.left_stick_x;
                rx = -gamepad1.right_stick_x * 0.75;
                frontLeftWheelDS.setPower(y + x + rx);
                backLeftWheelDS.setPower(y - x + rx);
                frontRightWheelDS.setPower(y - x - rx);
                backRightWheelDS.setPower(y + x - rx);
                // Put loop blocks here.

              	/*
                shoot logic:
                1. charge flywheels
                2. set final intake servo to 0
                (shoots 2 balls)
                3. set final intake servo to 20
                3. continously spin _1150RPMintake
                4. set final intake servo to 0
                (shoots last ball)
                5. stop _1150RPMintake and set final intake servo to 20

                note that the final intake servo takes ~1000ms to rotate between 20 and 0
                when moving forward, left flywheel is negative and right flywheel is pos
                */

                // in the original code, im pretty sure final intake servo automatically shoots in the next if statement below, so the first if that was here was essentially acting like a backup
                if (shoot == false) {
                    finalIntakeServo.setPosition(20);
                    FinalIntakeLeftDS.setPosition(20);
                }

                if (shoot) {
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            // tx, ty, and botpose telemetry data
                            Pose3D botpose = result.getBotpose();
                            telemetry.addData("tx", result.getTx());
                            telemetry.addData("ty", result.getTy());
                            telemetry.addData("Botpose", botpose.toString());

                            // april tag distance stuff

                            List<AprilTagDetection> detections = aprilTag.getDetections();
                            if (!detections.isEmpty()) {
                                AprilTagDetection detection = detections.get(0);
                                double distance = detection.ftcPose.z;

                                telemetry.addData("Tag Distance (in)", distance);
                            }
                            // if the result is valid, we're able to try auto-aiming

                            if (isAlignedWithTarget == false) {
                                // put logic here
                                // if close enough to target, set to true
                                // else
                                    // if tx is less than 0, move left
                                    // else
                                        // move right (because tx has to be greater than 0 if not less than 0
                            }
                            } else if (timer.milliseconds() < shootFirst) { // 500 ms gap between this and above if is risky, if shooting isn't working change this
                                finalIntakeServo.setPosition(0);
                                FinalIntakeLeftDS.setPosition(0);
                            } else if (timer.milliseconds() < prepareSecond) { // same comment as above
                                finalIntakeServo.setPosition(20);
                                FinalIntakeLeftDS.setPosition(20);
                                _1150RPMintake.setPower(-1);
                            } else if (timer.milliseconds() < shootSecond && Math.abs(_6000RPMmotor.getVelocity()) >= 1500) { // same comment as above
                                finalIntakeServo.setPosition(0);
                                finalIntakeServo.setPosition(0);
                                _1150RPMintake.setPower(0);
                            } else {
                                finalIntakeServo.setPosition(20);
                                FinalIntakeLeftDS.setPosition(20);
                                _1150RPMintake.setPower(0);
                                shoot = false;
                            }
                        }
                    else {
                        telemetry.addLine("*****2 No valid Limelight data !_!"); // in case something weird happens
                    }

                }


                if (gamepad1.right_bumper) {
                    _1150RPMintake.setPower(-1);
                }
                else if (gamepad1.a) {
                    _1150RPMintake.setPower(1);
                }
                else {
                    _1150RPMintake.setPower(0);
                }

                if (gamepad1.y) {
                    _6000RPMmotor.setVelocity(-1500);
                    _6000RPMmotorflywheelright.setVelocity(1500);

                    // only set shoot to true if you can see target
                    LLResult initialResult = limelight.getLatestResult();
                    if (initialResult != null) {
                        if (initialResult.isValid()) {
                            // auto detect rpm
                            if (Math.abs(_6000RPMmotor.getVelocity()) >= 1500 && shoot == false && timer.milliseconds() > shootGap) {
                                initialResult.getTx();
                                shoot = true;
                                timer.reset();
                            }
                        }
                    } else {
                        telemetry.addLine("No valid Limelight data !_!");
                    }
                } else {
                    _6000RPMmotor.setPower(0);
                    _6000RPMmotorflywheelright.setPower(0);
                    shoot = false;
                }
                if (gamepad1.x) {
                    _6000RPMmotor.setPower(0.635);
                    _6000RPMmotorflywheelright.setPower(-0.635);
                }
                PIDFCoefficients pidfCurrent = _6000RPMmotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                PIDFCoefficients pidfCurrent2 = _6000RPMmotorflywheelright.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("P", pidfCurrent.p);
                telemetry.addData("P2", pidfCurrent2.p);
                telemetry.addData("shoot:", shoot);
                telemetry.addData("Left Flywheel RPM: ", _6000RPMmotor.getVelocity()); // peak 1800, avg 1500
                telemetry.addData("Right Flywheel RPM: ", _6000RPMmotorflywheelright.getVelocity()); // peak 1800, avg 1500
                telemetry.update();
            }
        }
    }
}
