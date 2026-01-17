package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "OriginalCodeMoveBasedOnController6 (Blocks to Java)")
public class TeleopEfficientChassisMovement extends LinearOpMode {

    private Servo FinalIntakeLeftDS;
    private Servo finalIntakeServo;
    private DcMotor frontLeftWheelDS;
    private DcMotor backLeftWheelDS;
    private DcMotor frontRightWheelDS;
    private DcMotor backRightWheelDS;
    private DcMotor _1150RPMintake;
    private DcMotorEx _6000RPMmotor;
    private DcMotorEx _6000RPMmotorflywheelright;

    /**
     * idk man figure it out
     */
    @Override
    public void runOpMode() {
        float frontLeftPower;
        float backLeftPower;
        float frontRightPower;
        float backRightPower;
        double x, y, rx;

        FinalIntakeLeftDS = hardwareMap.get(Servo.class, "FinalIntakeLeftDS");
        finalIntakeServo = hardwareMap.get(Servo.class, "finalIntakeServo");
        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");
        _1150RPMintake = hardwareMap.get(DcMotor.class, "1150 RPM intake");
        _6000RPMmotor = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        _6000RPMmotorflywheelright = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");
        frontRightWheelDS.setDirection(DcMotorEx.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotorEx.Direction.REVERSE);

        // Put initialization blocks here.
        FinalIntakeLeftDS.setPosition(20);
        finalIntakeServo.setDirection(Servo.Direction.REVERSE);
        finalIntakeServo.setPosition(20);
        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(10.0, 3.0, 0.0, 12.0);
        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;
                frontLeftWheelDS.setPower(y + x + rx);
                backLeftWheelDS.setPower(y - x + rx);
                frontRightWheelDS.setPower(y - x - rx);
                backRightWheelDS.setPower(y + x - rx);
                // Put loop blocks here.
                if (gamepad1.left_bumper) {
                    FinalIntakeLeftDS.setPosition(0);
                } else {
                    FinalIntakeLeftDS.setPosition(20);
                }
                if (gamepad1.left_bumper) {
                    finalIntakeServo.setPosition(0);
                } else {
                    finalIntakeServo.setPosition(20);
                }

                if (gamepad1.a) {
                    frontLeftPower = 1;
                    backLeftPower = 1;
                    frontRightPower = 1;
                    backRightPower = 1;
                    frontLeftWheelDS.setPower(frontLeftPower);
                    backLeftWheelDS.setPower(backLeftPower);
                    frontRightWheelDS.setPower(frontRightPower);
                    backRightWheelDS.setPower(backRightPower);
                }
                if (gamepad1.b) {
                    frontLeftPower = 1;
                    backLeftPower = 1;
                    frontRightPower = 1;
                    backRightPower = 1;
                    frontLeftWheelDS.setPower(-frontLeftPower);
                    backLeftWheelDS.setPower(-backLeftPower);
                    frontRightWheelDS.setPower(-frontRightPower);
                    backRightWheelDS.setPower(-backRightPower);
                }
                if (gamepad1.x) {
                    _1150RPMintake.setPower(-1);
                } else {
                    _1150RPMintake.setPower(0);
                }
                if (gamepad1.dpad_up) {
                    // run front left motor
                    frontLeftWheelDS.setPower(1);
                }
                if (gamepad1.dpad_left) {
                    // run front left motor
                    backLeftWheelDS.setPower(1);
                }
                if (gamepad1.dpad_right) {
                    // run front left motor
                    frontRightWheelDS.setPower(1);
                }
                if (gamepad1.dpad_down) {
                    // run front left motor
                    backRightWheelDS.setPower(1);
                }
                if (gamepad1.y) {
                    _6000RPMmotor.setPower(-0.635);
                    _6000RPMmotorflywheelright.setPower(0.635);
                } else {
                    _6000RPMmotor.setPower(0);
                    _6000RPMmotorflywheelright.setPower(0);
                }
                if (gamepad1.right_bumper) {
                    _6000RPMmotor.setPower(0.635);
                    _6000RPMmotorflywheelright.setPower(-0.635);
                }
                PIDFCoefficients pidfCurrent = _6000RPMmotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                PIDFCoefficients pidfCurrent2 = _6000RPMmotorflywheelright.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("P", pidfCurrent.p);
                telemetry.addData("P2", pidfCurrent2.p);
                telemetry.update();
            }
        }
    }
}
