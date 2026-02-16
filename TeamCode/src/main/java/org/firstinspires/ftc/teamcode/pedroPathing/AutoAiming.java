package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "AutoAiming ----------------------------------")
public class AutoAiming extends LinearOpMode {

    private CRServo FinalIntakeLeftDS;
    private CRServo finalIntakeServo;

    private DcMotor frontLeftWheelDS;
    private DcMotor backLeftWheelDS;
    private DcMotor frontRightWheelDS;
    private DcMotor backRightWheelDS;

    private DcMotor _1150RPMintake;

    private DcMotorEx _6000RPMmotor;
    private DcMotorEx _6000RPMmotorflywheelright;

    private Limelight3A limelight;

    private double x, y, rx;

    private boolean shoot = false;
    private boolean shooterActive = false;
    private ElapsedTime timer = new ElapsedTime();

    private int shootGap = 2000;
    private int shootFirst = 500;
    private int prepareSecond = 1500;
    private int stopIntake = 2000;
    private int shootSecond = 2500;

    private static final double TICKS_PER_REV = 28;
    private static final double RPM_TOLERANCE = 100;

    private double SHOOT_RPM = 3100;
    private double TARGET_SHOOT_RPM = 3100;
    private double targetTicksPerSec; // always positive

    private int IntakeInward = -2;
    private int IntakeOutward = 1;

    private static final double F_Intake_Shoot = 1.0;
    private static final double F_Intake_Backwards = -1.0;
    private static final double F_Intake_Hold = 0.0;

    private final double[] TY_VALUES = {2.8, 5, 6.37, 10.0, 13.6, 17};
    private final double[] RPM_VALUES = {3100, 2930, 2600, 2525, 2375, 2300};

    private double getInterpolatedRPM(double ty) {
        if (ty <= TY_VALUES[0]) return RPM_VALUES[0];
        if (ty >= TY_VALUES[TY_VALUES.length - 1]) return RPM_VALUES[RPM_VALUES.length - 1];
        for (int i = 0; i < TY_VALUES.length - 1; i++) {
            double tyLow = TY_VALUES[i];
            double tyHigh = TY_VALUES[i + 1];
            if (ty >= tyLow && ty <= tyHigh) {
                double rpmLow = RPM_VALUES[i];
                double rpmHigh = RPM_VALUES[i + 1];
                double percent = (ty - tyLow) / (tyHigh - tyLow);
                return rpmLow + percent * (rpmHigh - rpmLow);
            }
        }
        return RPM_VALUES[0];
    }

    @Override
    public void runOpMode() {

        // --- hardware mapping ---
        FinalIntakeLeftDS = hardwareMap.get(CRServo.class, "FinalIntakeLeftDS");
        finalIntakeServo = hardwareMap.get(CRServo.class, "finalIntakeServo");

        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");

        _1150RPMintake = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        _6000RPMmotor = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        _6000RPMmotorflywheelright = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotor.Direction.REVERSE);

        // --- set motors to BRAKE by default ---
        frontLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        FinalIntakeLeftDS.setPower(F_Intake_Hold);
        FinalIntakeLeftDS.setDirection(CRServo.Direction.REVERSE);
        finalIntakeServo.setPower(F_Intake_Hold);

        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- PIDF for flywheel motors ---
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(0.011, 0.0, 0.001, 10);
        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        waitForStart();

        while (opModeIsActive()) {

            // --- driver input ---
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x * 0.75;

            LLResult result = limelight.getLatestResult();

            // --- auto-aim adjustments only update rx ---
            if (gamepad1.b && result != null && result.isValid()) {
                double tx = result.getTx();
                double absTx = Math.abs(tx);

                double alignedThreshold = 0.75;
                double minRx = 0.06;
                double maxRx = 0.2;

                if (absTx <= alignedThreshold) {
                    rx = 0;
                    for (int i = 0; i < 10; i++) {
                        telemetry.addLine("Aligned with target ----------");
                    }
                } else {
                    double scale;
                    if (absTx > 2) {
                        scale = maxRx;
                    } else {
                        scale = Math.pow(absTx / 2.0, 1.5) * maxRx;
                        if (scale < minRx) scale = minRx;
                    }
                    rx = -Math.signum(tx) * scale;
                }

                telemetry.addData("tx ------", tx);
                telemetry.addData("rx applied", rx);

                // --- BRAKE while B held ---
                frontLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                // FLOAT otherwise
                frontLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // --- drive motors ---
            frontLeftWheelDS.setPower(y + x + rx);
            backLeftWheelDS.setPower(y - x + rx);
            frontRightWheelDS.setPower(y - x - rx);
            backRightWheelDS.setPower(y + x - rx);

            // --- shooter/flywheel logic ---
            SHOOT_RPM = 3100;
            TARGET_SHOOT_RPM = 3100;

            if (result != null && result.isValid()) {
                double ty = result.getTy();
                TARGET_SHOOT_RPM = getInterpolatedRPM(ty);
                SHOOT_RPM = TARGET_SHOOT_RPM;
            }

            // --- use 50 RPM below target for shooting to prevent overshoot ---
            double adjustedShootRPM = TARGET_SHOOT_RPM - 50;
            targetTicksPerSec = adjustedShootRPM * TICKS_PER_REV / 60.0;

            double leftRPM = _6000RPMmotor.getVelocity() * 60.0 / TICKS_PER_REV;
            double rightRPM = _6000RPMmotorflywheelright.getVelocity() * 60.0 / TICKS_PER_REV;

            // --- flywheel idle / active control ---
            if (gamepad1.yWasPressed()) {
                shooterActive = true;
                timer.reset();
            }

            if (gamepad1.aWasPressed()) {
                shooterActive = false;
                shoot = false;
            }

            if (shooterActive) {
                _6000RPMmotor.setVelocity(-targetTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(targetTicksPerSec);
            } else {
                double idleTicksPerSec = 2400 * TICKS_PER_REV / 60.0; // 2400 RPM idle
                _6000RPMmotor.setVelocity(-idleTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(idleTicksPerSec);
            }

            // --- final intake idle ---
            if (!shoot) {
                finalIntakeServo.setPower(F_Intake_Hold);
                FinalIntakeLeftDS.setPower(F_Intake_Hold);
            }

            // --- shooting sequence ---
            if (shoot) {
                if (timer.milliseconds() < shootFirst) {
                    finalIntakeServo.setPower(F_Intake_Shoot);
                    FinalIntakeLeftDS.setPower(F_Intake_Shoot);
                } else if (timer.milliseconds() < prepareSecond) {
                    finalIntakeServo.setPower(F_Intake_Hold);
                    FinalIntakeLeftDS.setPower(F_Intake_Hold);
                    _1150RPMintake.setPower(IntakeInward);
                } else if (timer.milliseconds() < stopIntake) {
                    _1150RPMintake.setPower(0);
                } else if (timer.milliseconds() < shootSecond
                        && Math.abs(leftRPM) >= adjustedShootRPM - RPM_TOLERANCE
                        && Math.abs(leftRPM) <= adjustedShootRPM) {
                    finalIntakeServo.setPower(F_Intake_Shoot);
                    FinalIntakeLeftDS.setPower(F_Intake_Shoot);
                } else {
                    finalIntakeServo.setPower(F_Intake_Hold);
                    FinalIntakeLeftDS.setPower(F_Intake_Hold);
                    shoot = false;
                    timer.reset();
                }
            }

            // --- manual intake controls ---
            if (!shoot) {
                if (gamepad1.right_bumper) {
                    _1150RPMintake.setPower(IntakeInward);
                    finalIntakeServo.setPower(F_Intake_Backwards);
                    FinalIntakeLeftDS.setPower(F_Intake_Backwards);
                } else if (gamepad1.left_bumper) {
                    _1150RPMintake.setPower(IntakeOutward);
                } else {
                    _1150RPMintake.setPower(0);
                }
            }

            // --- telemetry ---
            telemetry.addData("shoot:", shoot);
            telemetry.addData("shooterActive:", shooterActive);
            telemetry.addData("Shoot RPM:", SHOOT_RPM);
            telemetry.addData("Target RPM:", TARGET_SHOOT_RPM);
            telemetry.addData("Adjusted Shoot RPM:", adjustedShootRPM);
            telemetry.addData("Left Flywheel RPM:", Math.round(leftRPM));
            telemetry.addData("Right Flywheel RPM:", Math.round(rightRPM));
            telemetry.addData("Final Intake Left Power", FinalIntakeLeftDS.getPower());
            telemetry.addData("Final Intake Right Power", finalIntakeServo.getPower());
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
            telemetry.update();
        }
    }
}
