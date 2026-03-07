package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult; // allows us to track apriltag ID
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "SFastShoot")
public class FastShoot extends LinearOpMode {

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
    private boolean activateFinalIntakeTemporary = false;
    private boolean reverseFlywheels = false;
    private ElapsedTime timer = new ElapsedTime();

    private static final double TICKS_PER_REV = 28;
    private double RPM_TOLERANCE = 200;
    private double TARGET_SHOOT_RPM = 3100;
    private double targetTicksPerSec;

    private int IntakeInward = -2;
    private int IntakeOutward = 1;

    private static final double F_Intake_Shoot = 1.0;
    private static final double F_Intake_Backwards = -0.25;
    private static final double F_Intake_Hold = 0.0;

    private final double[] TY_VALUES = {2.8, 5, 6.37, 10.0, 13.6, 17}; // 6.37 is at the edge of the front shooting zone
    private final double[] RPM_VALUES = {3050, 2880, 2510, 2325, 2175, 2100}; // short shots = add more, long shots = subtract more
    private final double[] RPM_TOLERANCE_VALUES = {0, 0, 0, 0, 0, 0}; // {0, 25, 50, 50, 60, 50}

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
    private double getInterpolatedToleranceRPM(double ty) {
        if (ty <= TY_VALUES[0]) return RPM_TOLERANCE_VALUES[0];
        if (ty >= TY_VALUES[TY_VALUES.length - 1]) return RPM_TOLERANCE_VALUES[RPM_TOLERANCE_VALUES.length - 1];
        for (int i = 0; i < TY_VALUES.length - 1; i++) {
            double tyLow = TY_VALUES[i];
            double tyHigh = TY_VALUES[i + 1];
            if (ty >= tyLow && ty <= tyHigh) {
                double rpmLow = RPM_TOLERANCE_VALUES[i];
                double rpmHigh = RPM_TOLERANCE_VALUES[i + 1];
                double percent = (ty - tyLow) / (tyHigh - tyLow);
                return rpmLow + percent * (rpmHigh - rpmLow);
            }
        }
        return RPM_TOLERANCE_VALUES[0];
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

        frontRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.start();

        FinalIntakeLeftDS.setPower(F_Intake_Hold);
        FinalIntakeLeftDS.setDirection(CRServo.Direction.REVERSE);
        finalIntakeServo.setPower(F_Intake_Hold);

        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- PIDF for flywheel motors --- // target 2500 2600
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(0.21, 0.001, 0.001, 13.5);
        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        waitForStart();

        while (opModeIsActive()) {

            // --- driver input ---
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x;

            LLResult result = limelight.getLatestResult();

            // --- auto-aim ---
            if (gamepad1.b && result != null && result.isValid()) {

                FiducialResult bestTag = null;
                double smallestAbsTx = Double.MAX_VALUE;

                for (FiducialResult tag : result.getFiducialResults()) {

                    if (tag.getFiducialId() == 20 || tag.getFiducialId() == 24) {

                        double absTx = Math.abs(tag.getTargetXDegrees());

                        if (absTx < smallestAbsTx) {
                            smallestAbsTx = absTx;
                            bestTag = tag;
                        }
                    }
                }

                if (bestTag != null) {
                    double tx = bestTag.getTargetXDegrees();
                    double absTx = Math.abs(tx);

                    double alignedThreshold = 0.75;
                    double minRx = 0.06;
                    double maxRx = 0.2;

                    if (absTx <= alignedThreshold) {
                        rx = 0;
                    } else {
                        double scale =
                                (absTx > 2)
                                        ? maxRx
                                        : Math.pow(absTx / 2.0, 1.5) * maxRx;

                        if (scale < minRx) scale = minRx;

                        rx = -Math.signum(tx) * scale;
                    }
                } else {
                    rx = 0;
                }
            }

            // --- drive ---
            frontLeftWheelDS.setPower(y + x + rx);
            backLeftWheelDS.setPower(y - x + rx);
            frontRightWheelDS.setPower(y - x - rx);
            backRightWheelDS.setPower(y + x - rx);

            // --- target RPM ---
            TARGET_SHOOT_RPM = 3100;
            if (result != null && result.isValid()) {
                FiducialResult bestTag = null;
                double smallestAbsTx = Double.MAX_VALUE;

                for (FiducialResult tag : result.getFiducialResults()) {

                    if (tag.getFiducialId() == 20 || tag.getFiducialId() == 24) {

                        double absTx = Math.abs(tag.getTargetXDegrees());

                        if (absTx < smallestAbsTx) {
                            smallestAbsTx = absTx;
                            bestTag = tag;
                        }
                    }
                }

                if (bestTag != null) {
                    TARGET_SHOOT_RPM = getInterpolatedRPM(bestTag.getTargetYDegrees());
                }
            }

            if (result != null && result.isValid()) {
                FiducialResult bestTag = null;
                double smallestAbsTx = Double.MAX_VALUE;

                for (FiducialResult tag : result.getFiducialResults()) {

                    if (tag.getFiducialId() == 20 || tag.getFiducialId() == 24) {

                        double absTx = Math.abs(tag.getTargetXDegrees());

                        if (absTx < smallestAbsTx) {
                            smallestAbsTx = absTx;
                            bestTag = tag;
                        }
                    }
                }

                if (bestTag != null && result.isValid()) {
                    RPM_TOLERANCE = getInterpolatedToleranceRPM(bestTag.getTargetYDegrees());
                }
            }
            targetTicksPerSec = TARGET_SHOOT_RPM * TICKS_PER_REV / 60.0;

            double leftRPM = _6000RPMmotor.getVelocity() * 60.0 / TICKS_PER_REV;
            double rightRPM = _6000RPMmotorflywheelright.getVelocity() * 60.0 / TICKS_PER_REV;

            // --- controls ---
            if (gamepad1.yWasPressed()) {
                shooterActive = true;
                shoot = true;      // start shooting sequence
                timer.reset();
            }

            if (gamepad1.xWasPressed()) {
                reverseFlywheels = true;
                shooterActive = false;
                shoot = false;
                _6000RPMmotor.setPower(0.45);          // left reversed
                _6000RPMmotorflywheelright.setPower(-0.45); // right reversed
            }

            if (gamepad1.a) {
                reverseFlywheels = false;
                shooterActive = false;
                shoot = false;
                _6000RPMmotor.setPower(0.025); // spin slightly backwards so you can reload and shoot faster next time
                _6000RPMmotorflywheelright.setPower(-0.025); // spin slightly backwards so you can reload and shoot faster next time
            }
            else if (!reverseFlywheels) {
                _6000RPMmotor.setPower(-0.45);
                _6000RPMmotorflywheelright.setPower(0.45);
            }



            // --- DPad Down: Limelight telemetry ---
            if (gamepad1.dpad_down && result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }

            if (!shoot) {
                if (gamepad1.right_bumper) {
                    _1150RPMintake.setPower(IntakeInward);
                    finalIntakeServo.setPower(F_Intake_Backwards);
                    FinalIntakeLeftDS.setPower(F_Intake_Backwards);
                } else if (gamepad1.left_bumper) {
                    _1150RPMintake.setPower(IntakeOutward);
                    finalIntakeServo.setPower(F_Intake_Backwards);
                    FinalIntakeLeftDS.setPower(F_Intake_Backwards);
                } else {
                    _1150RPMintake.setPower(0);
                    finalIntakeServo.setPower(F_Intake_Hold);
                    FinalIntakeLeftDS.setPower(F_Intake_Hold);
                }
            }

            // --- shooting sequence ---
            if (shoot) {
                reverseFlywheels = false;

                // check if flywheels are within tolerance
                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RPM_TOLERANCE && Math.abs(leftRPM) <= TARGET_SHOOT_RPM + 300) {

                    // spin flywheels toward target
                    _6000RPMmotor.setPower(-0.8);
                    _6000RPMmotorflywheelright.setPower(0.8);

                    // flywheels are ready → activate final intake to shoot
                    activateFinalIntakeTemporary = true;
                    finalIntakeServo.setPower(F_Intake_Shoot);
                    FinalIntakeLeftDS.setPower(F_Intake_Shoot);

                    // optionally, run main intake if needed in intake mode
                    _1150RPMintake.setPower(IntakeInward);
                } else if (Math.abs(leftRPM) > TARGET_SHOOT_RPM + 300) {
                    _6000RPMmotor.setPower(0.05);
                    _6000RPMmotorflywheelright.setPower(-0.05);
                    _1150RPMintake.setPower(0);
                    activateFinalIntakeTemporary = false;
                }
                else {
                    // spin flywheels toward target
                    _6000RPMmotor.setPower(-1);
                    _6000RPMmotorflywheelright.setPower(1);

                    // flywheels not at target → stop intake and final intake
                    finalIntakeServo.setPower(F_Intake_Hold);
                    FinalIntakeLeftDS.setPower(F_Intake_Hold);
                    _1150RPMintake.setPower(0);
                    activateFinalIntakeTemporary = false;
                }
            }

            // --- telemetry ---
            telemetry.addData("shoot:", shoot);
            telemetry.addData("shooterActive:", shooterActive);
            telemetry.addData("Target RPM:", TARGET_SHOOT_RPM);
            telemetry.addData("Left Flywheel RPM:", Math.round(leftRPM));
            telemetry.addData("Right Flywheel RPM:", Math.round(rightRPM));
            telemetry.addData("Activate Flywheels:", activateFinalIntakeTemporary);
            telemetry.update();
        }
    }
}
