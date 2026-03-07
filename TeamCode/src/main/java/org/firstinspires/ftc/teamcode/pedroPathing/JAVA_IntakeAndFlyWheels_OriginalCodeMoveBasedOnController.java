package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OriginalCodeMoveBasedOnController6 (Blocks to Java)")
public class JAVA_IntakeAndFlyWheels_OriginalCodeMoveBasedOnController extends LinearOpMode {

    private CRServo FinalIntakeLeftDS;
    private CRServo finalIntakeServo;
    private DcMotor frontLeftWheelDS;
    private DcMotor backLeftWheelDS;
    private DcMotor frontRightWheelDS;
    private DcMotor backRightWheelDS;
    private DcMotor _1150RPMintake;
    private DcMotor _6000RPMmotor;
    private DcMotor _6000RPMmotorflywheelright;

    /**
     * idk man figure it out
     */
    @Override
    public void runOpMode() {
        float frontLeftPower;
        float LY;
        float backLeftPower;
        float frontRightPower;
        float backRightPower;

        FinalIntakeLeftDS = hardwareMap.get(CRServo.class, "FinalIntakeLeftDS");
        finalIntakeServo = hardwareMap.get(CRServo.class, "finalIntakeServo");
        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");
        _1150RPMintake = hardwareMap.get(DcMotor.class, "1150 RPM intake");
        _6000RPMmotor = hardwareMap.get(DcMotor.class, "6000 RPM motor");
        _6000RPMmotorflywheelright = hardwareMap.get(DcMotor.class, "6000 RPM motor flywheel right");

        // Put initialization blocks here.
        finalIntakeServo.setDirection(CRServo.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.dpad_up) {
                    frontLeftWheelDS.setPower(1); //front left
                }
                else {
                    frontLeftWheelDS.setPower(0);
                }

                if (gamepad1.dpad_left) {
                    backLeftWheelDS.setPower(1); // back left
                }
                else {
                    backLeftWheelDS.setPower(0);
                }

                if (gamepad1.dpad_down) {
                    frontRightWheelDS.setPower(1); // front right
                }
                else {
                    frontRightWheelDS.setPower(0);
                }


                if (gamepad1.dpad_right) {
                    backRightWheelDS.setPower(1); // back right
                }
                else {
                    backRightWheelDS.setPower(0);
                }


                // original code


                if (gamepad1.left_bumper) {
                    FinalIntakeLeftDS.setPower(-1);
                    finalIntakeServo.setPower(1);
                } else {
                    FinalIntakeLeftDS.setPower(0);
                    finalIntakeServo.setPower(0);
                }

                /*

                LY = gamepad1.left_stick_y;
                telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
                telemetry.addData("Left Stick X", gamepad1.left_stick_y);
                // Front Left Wheel
                // Forward + Backward
                frontLeftPower = gamepad1.left_stick_y;
                // Front Left Wheel
                // Forward + Backward
                backLeftPower = gamepad1.left_stick_y;
                // Front Left Wheel
                // Forward + Backward
                frontRightPower = -gamepad1.left_stick_y;
                // Front Left Wheel
                // Forward + Backward
                backRightPower = -gamepad1.left_stick_y;
                frontLeftWheelDS.setPower(frontLeftPower);
                telemetry.addData("key", frontLeftPower);
                telemetry.addData("key", frontLeftWheelDS.getPower());
                backLeftWheelDS.setPower(backLeftPower);
                telemetry.addData("key", backLeftPower);
                telemetry.addData("key", backLeftWheelDS.getPower());
                frontRightWheelDS.setPower(frontRightPower);
                telemetry.addData("key", frontRightPower);
                telemetry.addData("key", frontRightWheelDS.getPower());
                backRightWheelDS.setPower(backRightPower);
                telemetry.addData("key", backRightPower);
                telemetry.addData("key", backRightWheelDS.getPower());
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Left + Right
                // Front Left Wheel
                // Forward + Backward
                frontLeftPower = gamepad1.left_stick_x;
                // Front Left Wheel
                // Forward + Backward
                backLeftPower = -gamepad1.left_stick_x;
                // Front Left Wheel
                // Forward + Backward
                frontRightPower = -gamepad1.left_stick_x;
                // Front Left Wheel
                // Forward + Backward
                backRightPower = gamepad1.left_stick_x;
                frontLeftWheelDS.setPower(frontLeftPower);
                telemetry.addData("key", frontLeftPower);
                telemetry.addData("key", frontLeftWheelDS.getPower());
                backLeftWheelDS.setPower(backLeftPower);
                telemetry.addData("key", backLeftPower);
                telemetry.addData("key", backLeftWheelDS.getPower());
                frontRightWheelDS.setPower(frontRightPower);
                telemetry.addData("key", frontRightPower);
                telemetry.addData("key", frontRightWheelDS.getPower());
                backRightWheelDS.setPower(backRightPower);
                telemetry.addData("key", backRightPower);
                telemetry.addData("key", backRightWheelDS.getPower());
                // Turning
                // Turning
                // Turning
                // Turning
                // Turning
                // Turning
                // Turning
                // Turning
                // Turning
                // Turning
                // Front Left Wheel
                // Forward + Backward
                frontLeftPower = -gamepad1.right_stick_x;
                // Front Left Wheel
                // Forward + Backward
                backLeftPower = -gamepad1.right_stick_x;
                // Front Left Wheel
                // Forward + Backward
                frontRightPower = -gamepad1.right_stick_x;
                // Front Left Wheel
                // Forward + Backward
                backRightPower = -gamepad1.right_stick_x;
                frontLeftWheelDS.setPower(frontLeftPower);
                telemetry.addData("key", frontLeftPower);
                telemetry.addData("key", frontLeftWheelDS.getPower());
                backLeftWheelDS.setPower(backLeftPower);
                telemetry.addData("key", backLeftPower);
                telemetry.addData("key", backLeftWheelDS.getPower());
                frontRightWheelDS.setPower(frontRightPower);
                telemetry.addData("key", frontRightPower);
                telemetry.addData("key", frontRightWheelDS.getPower());
                backRightWheelDS.setPower(backRightPower);
                telemetry.addData("key", backRightPower);
                telemetry.addData("key", backRightWheelDS.getPower());
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

                 */
                if (gamepad1.x) {
                    _1150RPMintake.setPower(-1);
                } else {
                    _1150RPMintake.setPower(0);
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
                telemetry.update();
            }
        }
    }
}
