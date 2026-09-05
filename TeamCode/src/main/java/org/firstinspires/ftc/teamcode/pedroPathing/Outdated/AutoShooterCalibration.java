//package org.firstinspires.ftc.teamcode.pedroPathing.Outdated;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//@TeleOp(name = "Auto Shooter Calibration", group = "Test")
//public class AutoShooterCalibration extends LinearOpMode {
//
//    private Servo FinalIntakeLeftDS;
//    private Servo finalIntakeServo;
//
//    private DcMotor frontLeftWheelDS;
//    private DcMotor backLeftWheelDS;
//    private DcMotor frontRightWheelDS;
//    private DcMotor backRightWheelDS;
//
//    private DcMotor _1150RPMintake;
//    private DcMotorEx _6000RPMmotor;
//    private DcMotorEx _6000RPMmotorflywheelright;
//
//    private Limelight3A limelight;
//
//    private double x, y, rx;
//
//    private boolean shoot = false;
//    private ElapsedTime timer = new ElapsedTime();
//
//    private final int shootGap = 2000;
//    private final int shootFirst = 500;
//    private final int prepareSecond = 1500;
//    private final int stopIntake = 2000;
//    private final int shootSecond = 2500;
//
//    private final int IntakeInward = -2;
//    private final int IntakeOutward = 1;
//
//    private static final double TICKS_PER_REV = 28;
//    private static final double RPM_TOLERANCE = 100;
//
//    // Manual adjustable RPM
//    private double targetRPM = 3100;
//
//    @Override
//    public void runOpMode() {
//
//        // ----------- Hardware Map -----------
//
//        FinalIntakeLeftDS = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
//        finalIntakeServo = hardwareMap.get(Servo.class, "finalIntakeServo");
//
//        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
//        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
//        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
//        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");
//
//        _1150RPMintake = hardwareMap.get(DcMotor.class, "1150 RPM intake");
//
//        _6000RPMmotor = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
//        _6000RPMmotorflywheelright =
//                hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        frontRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
//        backRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
//        finalIntakeServo.setDirection(Servo.Direction.REVERSE);
//
//        FinalIntakeLeftDS.setPosition(20);
//        finalIntakeServo.setPosition(20);
//
//        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        PIDFCoefficients shooterPIDF =
//                new PIDFCoefficients(0.015, 0.0, 0.001, 15);
//
//        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
//        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
//
//        limelight.pipelineSwitch(0);
//        limelight.start();
//
//        telemetry.setMsTransmissionInterval(11);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // ---------------- DRIVE ----------------
//
//            y = gamepad1.left_stick_y;
//            x = -gamepad1.left_stick_x;
//            rx = -gamepad1.right_stick_x * 0.75;
//
//            if (gamepad1.b) {
//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    if (Math.abs(result.getTx()) > 1) {
//                        rx = -(result.getTx() / 30);
//                        Pose3D botpose = result.getBotpose();
//                        telemetry.addData("tx", result.getTx());
//                        telemetry.addData("ty", result.getTy());
//                        telemetry.addData("Botpose", botpose);
//                    } else {
//                        telemetry.addLine("Aligned with target ----------");
//                    }
//                }
//            }
//
//            frontLeftWheelDS.setPower(y + x + rx);
//            backLeftWheelDS.setPower(y - x + rx);
//            frontRightWheelDS.setPower(y - x - rx);
//            backRightWheelDS.setPower(y + x - rx);
//
//            // ---------------- MANUAL RPM ADJUST ----------------
//
//            if (gamepad1.dpad_right) {
//                targetRPM += 50;
//                sleep(150);
//            }
//            if (gamepad1.dpad_up) {
//                targetRPM += 5;
//                sleep(150);
//            }
//            if (gamepad1.dpad_left) {
//                targetRPM -= 50;
//                sleep(150);
//            }
//            if (gamepad1.dpad_down) {
//                targetRPM -= 5;
//                sleep(150);
//            }
//
//            double shootTicksPerSec = targetRPM * TICKS_PER_REV / 60.0;
//
//            // ---------------- SHOOT LOGIC (UNCHANGED) ----------------
//
//            double leftRPM = _6000RPMmotor.getVelocity() * 60.0 / TICKS_PER_REV;
//
//            if (!shoot) {
//                finalIntakeServo.setPosition(20);
//                FinalIntakeLeftDS.setPosition(20);
//            }
//
//            if (shoot) {
//
//                if (timer.milliseconds() < shootFirst) {
//                    finalIntakeServo.setPosition(0);
//                    FinalIntakeLeftDS.setPosition(0);
//
//                } else if (timer.milliseconds() < prepareSecond) {
//                    finalIntakeServo.setPosition(20);
//                    FinalIntakeLeftDS.setPosition(20);
//                    _1150RPMintake.setPower(IntakeInward);
//
//                } else if (timer.milliseconds() < stopIntake) {
//                    _1150RPMintake.setPower(0);
//
//                } else if (timer.milliseconds() < shootSecond
//                        && Math.abs(leftRPM) >= targetRPM - RPM_TOLERANCE) {
//                    finalIntakeServo.setPosition(0);
//                    FinalIntakeLeftDS.setPosition(0);
//
//                } else {
//                    finalIntakeServo.setPosition(20);
//                    FinalIntakeLeftDS.setPosition(20);
//                    _6000RPMmotor.setVelocity(0);
//                    _6000RPMmotorflywheelright.setVelocity(0);
//                    shoot = false;
//                    timer.reset();
//                }
//            }
//
//            if (!shoot) {
//                if (gamepad1.right_bumper) _1150RPMintake.setPower(IntakeInward);
//                else if (gamepad1.left_bumper) _1150RPMintake.setPower(IntakeOutward);
//                else _1150RPMintake.setPower(0);
//            }
//
//            if (gamepad1.yWasReleased()) {
//                timer.reset();
//                _6000RPMmotor.setVelocity(-shootTicksPerSec);
//                _6000RPMmotorflywheelright.setVelocity(shootTicksPerSec);
//            }
//
//            if (gamepad1.aWasPressed()) {
//                _6000RPMmotor.setVelocity(0);
//                _6000RPMmotorflywheelright.setVelocity(0);
//                shoot = false;
//            }
//
//            if (Math.abs(leftRPM) >= targetRPM - RPM_TOLERANCE
//                    && !shoot
//                    && timer.milliseconds() > shootGap) {
//                shoot = true;
//                timer.reset();
//            }
//
//            if (gamepad1.x) {
//                shoot = false;
//                _6000RPMmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                _6000RPMmotor.setPower(0.3);
//                _6000RPMmotorflywheelright.setPower(-0.3);
//            }
//
//            // ---------------- TELEMETRY ----------------
//
//            telemetry.addLine("=== AUTO SHOOT MANUAL RPM ===");
//            LLResult result2 = limelight.getLatestResult();
//            if (result2 != null && result2.isValid()) {
//                telemetry.addData("ty", result2.getTy());
//            }
//            telemetry.addData("Target RPM", targetRPM);
//            telemetry.addData("Left RPM", Math.round(leftRPM));
//            telemetry.addData("Shooting", shoot);
//            telemetry.addLine("D-pad Up/Down = Adjust RPM");
//            telemetry.update();
//        }
//    }
//}