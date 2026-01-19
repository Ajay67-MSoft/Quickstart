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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "AutoShootTest")
public class AutoShootTest extends LinearOpMode {

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
    private int prepareSecond = 2000;
    private int shootSecond = 2500;
    private double TICKS_PER_REV;
    private double SHOOT_RPM;
    private double TARGET_SHOOT_RPM;
    private double shootTicksPerSec;
    private static final double RPM_TOLERANCE = 100; // RPM
    private int IntakeInward = -1;
    private int IntakeOutward = 1;
    private int IntakeNoPower = 0;

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

        PIDFCoefficients shooterPIDF =
                new PIDFCoefficients(0.003, 0.0, 0.0001, 11);

        _6000RPMmotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        _6000RPMmotorflywheelright.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

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

                TICKS_PER_REV = 28;
                SHOOT_RPM = 5000;
                TARGET_SHOOT_RPM = 2800;

                shootTicksPerSec = SHOOT_RPM * TICKS_PER_REV / 60.0;

                double leftRPM = _6000RPMmotor.getVelocity() * 60.0 / TICKS_PER_REV;
                double rightRPM = _6000RPMmotorflywheelright.getVelocity() * 60.0 / TICKS_PER_REV;

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
                            Pose3D botpose = result.getBotpose();
                            telemetry.addData("tx", result.getTx());
                            telemetry.addData("ty", result.getTy());
                            telemetry.addData("Botpose", botpose.toString());
                        }
                    }
                    else {
                        telemetry.addLine("No valid Limelight data !_!");
                    }

                    if (timer.milliseconds() < shootFirst) {  // 500 ms gap between this and above if is risky, if shooting isn't working change this
                        finalIntakeServo.setPosition(0);
                        FinalIntakeLeftDS.setPosition(0);
                    } else if (timer.milliseconds() < prepareSecond) { // same comment as above
                        finalIntakeServo.setPosition(20);
                        FinalIntakeLeftDS.setPosition(20);
                        _1150RPMintake.setPower(IntakeInward);
                    } else if (timer.milliseconds() < shootSecond && Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE) { // same comment as above
                        finalIntakeServo.setPosition(0);
                        FinalIntakeLeftDS.setPosition(0);
                        _1150RPMintake.setPower(0);
                    } else {
                        finalIntakeServo.setPosition(20);
                        FinalIntakeLeftDS.setPosition(20);
                        _6000RPMmotor.setVelocity(0);
                        _6000RPMmotorflywheelright.setVelocity(0);
                        shoot = false;
                        timer.reset();
                    }
                }

                if (!shoot) {
                    if (gamepad1.right_bumper) {
                        _1150RPMintake.setPower(IntakeInward);
                    } else if (gamepad1.a) {
                        _1150RPMintake.setPower(IntakeOutward);
                    } else {
                        _1150RPMintake.setPower(0);
                    }
                }

                if (gamepad1.yWasReleased()) {
                    _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    timer.reset();

                    _6000RPMmotor.setVelocity(-shootTicksPerSec);
                    _6000RPMmotorflywheelright.setVelocity(shootTicksPerSec);
                }

                if (gamepad1.aWasPressed()) {
                    _6000RPMmotor.setVelocity(0);
                    _6000RPMmotorflywheelright.setVelocity(0);
                    shoot = false;
                }

                // auto detect rpm
                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE
                        && !shoot
                        && timer.milliseconds() > shootGap) {
                    shoot = true;
                    timer.reset();
                }

                if (gamepad1.x) {
                    shoot = false;

                    // Emergency open-loop override
                    _6000RPMmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    _6000RPMmotor.setVelocity(shootTicksPerSec);
                    _6000RPMmotorflywheelright.setVelocity(-shootTicksPerSec);
                }


                double roundedLeftRPM  = Math.round(leftRPM / 20.0) * 20.0;
                double roundedRightRPM = Math.round(rightRPM / 20.0) * 20.0;

                PIDFCoefficients pidfCurrent = _6000RPMmotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                PIDFCoefficients pidfCurrent2 = _6000RPMmotorflywheelright.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("P", pidfCurrent.p);
                telemetry.addData("P2", pidfCurrent2.p);
                telemetry.addData("shoot:", shoot);
                telemetry.addData("Left Flywheel RPM:", roundedLeftRPM);
                telemetry.addData("Right Flywheel RPM:", roundedRightRPM);
                telemetry.addLine("v1");
                telemetry.update();
            }
        }
    }
}
