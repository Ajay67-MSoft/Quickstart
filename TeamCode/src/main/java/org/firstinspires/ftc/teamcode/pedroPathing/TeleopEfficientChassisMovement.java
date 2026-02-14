//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp(name = "TeleopEfficientChassisMovement")
//public class TeleopEfficientChassisMovement extends LinearOpMode {
//
//    private Servo FinalIntakeLeftDS;
//    private Servo finalIntakeServo;
//    private DcMotor frontLeftWheelDS;
//    private DcMotor backLeftWheelDS;
//    private DcMotor frontRightWheelDS;
//    private DcMotor backRightWheelDS;
//    private DcMotor _1150RPMintake;
//    private DcMotorEx _6000RPMmotor;
//    private DcMotorEx _6000RPMmotorflywheelright;
//    private double x, y, rx;
//    private boolean shoot = false; //double t = stateTimer.getElapsedTimeSeconds();
//    private ElapsedTime timer = new ElapsedTime();
//    private int shootTime = 400;
//
//    /**
//     * idk man figure it out
//     */
//    @Override
//    public void runOpMode() {
//
//        FinalIntakeLeftDS = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
//        finalIntakeServo = hardwareMap.get(Servo.class, "finalIntakeServo");
//        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
//        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
//        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
//        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");
//        _1150RPMintake = hardwareMap.get(DcMotor.class, "1150 RPM intake");
//        _6000RPMmotor = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
//        _6000RPMmotorflywheelright = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
//        frontRightWheelDS.setDirection(DcMotorEx.Direction.REVERSE);
//        backRightWheelDS.setDirection(DcMotorEx.Direction.REVERSE);
//
//        // Put initialization blocks here.
//        FinalIntakeLeftDS.setPosition(20);
//        finalIntakeServo.setDirection(Servo.Direction.REVERSE);
//        finalIntakeServo.setPosition(20);
//        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfNew = new PIDFCoefficients(10.0, 3.0, 0.0, 12.0);
//        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
//        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
//        waitForStart();
//        if (opModeIsActive()) {
//            // Put run blocks here.
//            while (opModeIsActive()) {
//                y = gamepad1.left_stick_y;
//                x = -gamepad1.left_stick_x;
//                rx = -gamepad1.right_stick_x;
//                frontLeftWheelDS.setPower(y + x + rx);
//                backLeftWheelDS.setPower(y - x + rx);
//                frontRightWheelDS.setPower(y - x - rx);
//                backRightWheelDS.setPower(y + x - rx);
//                // Put loop blocks here.
//
//              	/*
//                shoot logic:
//                1. charge flywheels
//                2. set final intake servo to 0
//                (shoots 2 balls)
//                3. set final intake servo to 20
//                3. continously spin _1150RPMintake
//                4. set final intake servo to 0
//                (shoots last ball)
//                5. stop _1150RPMintake and set final intake servo to 20
//
//                note that the final intake servo takes ~1000ms to rotate between 20 and 0
//                */
//
//                if (gamepad1.left_bumper && shoot == true) {
//                    finalIntakeServo.setPosition(0);
//                } else {
//                    finalIntakeServo.setPosition(20);
//                }
//
//                if (gamepad1.right_bumper) {
//                    _1150RPMintake.setPower(-1);
//                } else {
//                    _1150RPMintake.setPower(0);
//                }
//
//                if (gamepad1.y) {
//                    _6000RPMmotor.setVelocity(-1800);
//                    _6000RPMmotorflywheelright.setVelocity(1800);
//                    // auto detect rpm
//                    if (Math.abs(_6000RPMmotor.getVelocity()) >= 1500 && Math.abs(_6000RPMmotorflywheelright.getVelocity()) >= 1500) {
//                        shoot = true;
//                    } else {
//                        shoot = false;
//                    }
//                } else {
//                    _6000RPMmotor.setPower(0);
//                    _6000RPMmotorflywheelright.setPower(0);
//                    shoot = false;
//                }
//                if (gamepad1.x) {
//                    _6000RPMmotor.setPower(0.635);
//                    _6000RPMmotorflywheelright.setPower(-0.635);
//                }
//                PIDFCoefficients pidfCurrent = _6000RPMmotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//                PIDFCoefficients pidfCurrent2 = _6000RPMmotorflywheelright.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//                telemetry.addData("P", pidfCurrent.p);
//                telemetry.addData("P2", pidfCurrent2.p);
//                telemetry.addData("shoot:", shoot);
//                telemetry.addData("Left Flywheel RPM: ", _6000RPMmotor.getVelocity()); // peak 1800, avg 1500
//                telemetry.addData("Right Flywheel RPM: ", _6000RPMmotorflywheelright.getVelocity()); // peak 1800, avg 1500
//                telemetry.addLine("v2");
//                telemetry.update();
//            }
//        }
//    }
//}
