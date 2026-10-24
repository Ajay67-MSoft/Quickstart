package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "teleop_v1")
public class teleop_v1 extends LinearOpMode {

    private static final double SLOW_MODE_SCALE = 0.35; // speed while right trigger is held

    private DcMotor frontLeftWheelDS;
    private DcMotor backLeftWheelDS;
    private DcMotor frontRightWheelDS;
    private DcMotor backRightWheelDS;

    @Override
    public void runOpMode() {

        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");

        frontRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Teleop Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y  = -Math.pow(gamepad1.left_stick_y, 3);
            double x  =  Math.pow(gamepad1.left_stick_x, 3);
            double rx =  Math.pow(gamepad1.right_stick_x, 3);

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            // If any wheel power goes past 1.0, scale all four down so the robot keeps driving in the intended direction.
            double max = Math.max(
                    1.0,
                    Math.max(
                        Math.max(Math.abs(fl), Math.abs(bl)),
                        Math.max(Math.abs(fr), Math.abs(br))
                    )
            );

            double scale = (gamepad1.right_trigger > 0.5)
                    ? SLOW_MODE_SCALE
                    : 1.0;

            frontLeftWheelDS.setPower(fl / max * scale);
            backLeftWheelDS.setPower(bl / max * scale);
            frontRightWheelDS.setPower(fr / max * scale);
            backRightWheelDS.setPower(br / max * scale);

            telemetry.addData("Slow mode", scale < 1.0);
            telemetry.update();
        }
    }
}