package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Refactored TeleOp OpMode for Auto Aiming and Shooting.
 * This class uses RobotConfig.java to manage constants, making it easier to share
 * parameters with other programs (like Autonomous) and tune values via CSV.
 * 
 * Features:
 * - Dynamic Spooling: Flywheels adjust to calculated distance-based RPM automatically.
 * - Burst Fire: Dual-tolerance logic ensures two balls fire together consistently.
 * - Shooting Boost: Adds extra RPM during firing to compensate for kinetic energy loss.
 * - CSV Support: Loads tuning data from /sdcard/FIRST/shooter_lookup.csv.
 */
@TeleOp(name = "AutoAimingC (Configurable) -------------------")
public class AutoAimingC extends LinearOpMode {

    // --- Hardware Declarations ---
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

    // Drive variables
    private double x, y, rx;

    // State variables
    private boolean shoot = false;
    private boolean shooterActive = false;
    private boolean reverseFlywheels = false;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime(); // Timer to handle shooting duration/oscillation

    private double targetTicksPerSec;

    @Override
    public void runOpMode() {
        // Load external configuration from CSV if present (/sdcard/FIRST/shooter_lookup.csv)
        RobotConfig.loadLookupTable();

        // --- Hardware Mapping ---
        FinalIntakeLeftDS = hardwareMap.get(CRServo.class, "FinalIntakeLeftDS");
        finalIntakeServo = hardwareMap.get(CRServo.class, "finalIntakeServo");

        frontLeftWheelDS = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
        backLeftWheelDS = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
        frontRightWheelDS = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
        backRightWheelDS = hardwareMap.get(DcMotor.class, "backRightWheelDS");

        _1150RPMintake = hardwareMap.get(DcMotor.class, "1150 RPM intake");

        // Fixed hardware names to match actual robot configuration
        _6000RPMmotor = hardwareMap.get(DcMotorEx.class, "6000 RPM motor");
        _6000RPMmotorflywheelright = hardwareMap.get(DcMotorEx.class, "6000 RPM motor flywheel right");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set motor directions for proper chassis movement
        frontRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotor.Direction.REVERSE);

        // Allow motors to spin freely when power is 0 (prevents jerky stops)
        frontLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initial servo states using Config values
        FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
        FinalIntakeLeftDS.setDirection(CRServo.Direction.REVERSE);
        finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);

        // Configure flywheel motors for velocity control
        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply shared PIDF coefficients
        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, RobotConfig.SHOOTER_PIDF);
        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, RobotConfig.SHOOTER_PIDF);

        // Initialize shootTimer
        shootTimer.reset();

        waitForStart();

        while (opModeIsActive()) {

            // --- Driver Input ---
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x * 0.75;

            LLResult result = limelight.getLatestResult();

            // --- Auto-Aim Logic (Hold B Button) ---
            if (gamepad1.b && result != null && result.isValid()) {
                double tx = result.getTx();
                double absTx = Math.abs(tx);
                double alignedThreshold = RobotConfig.AIM_TOLERANCE_DEGREES;
                double minRx = RobotConfig.AIM_MIN_ROTATION_POWER;
                double maxRx = RobotConfig.AIM_MAX_ROTATION_POWER;

                if (absTx <= alignedThreshold) {
                    rx = 0;
                } else {
                    double scale = (absTx > 2) ? maxRx : Math.pow(absTx / 2.0, 1.5) * maxRx;
                    if (scale < minRx) scale = minRx;
                    rx = -Math.signum(tx) * scale;
                }
            }

            // --- Drive Execution ---
            frontLeftWheelDS.setPower(y + x + rx);
            backLeftWheelDS.setPower(y - x + rx);
            frontRightWheelDS.setPower(y - x - rx);
            backRightWheelDS.setPower(y + x - rx);

            // --- RPM Calculation & Dynamic Spooling ---
            double interpolatedRPM = RobotConfig.getInterpolatedRPM(result != null ? result.getTy() : 0);
            double currentGoalRPM;

            if (result != null && result.isValid()) {
                // Background spooling: interpolated RPM + small safety offset
                currentGoalRPM = Math.max(interpolatedRPM + RobotConfig.SHOOTER_SPOOL_OFFSET_RPM, RobotConfig.SHOOTER_MIN_ACTIVE_RPM);
            } else {
                // No target, stay at IDLE_RPM
                currentGoalRPM = RobotConfig.SHOOTER_IDLE_RPM;
            }

            targetTicksPerSec = currentGoalRPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;

            double leftRPM = _6000RPMmotor.getVelocity() * 60.0 / RobotConfig.SHOOTER_TICKS_PER_REV;
            double rightRPM = _6000RPMmotorflywheelright.getVelocity() * 60.0 / RobotConfig.SHOOTER_TICKS_PER_REV;

            // --- Controls ---
            if (gamepad1.yWasPressed()) {
                shooterActive = true;
                shoot = true;
                reverseFlywheels = false;
                timer.reset();
            }

            if (gamepad1.aWasPressed()) {
                shooterActive = false;
                shoot = false;
                reverseFlywheels = false;
            }

            if (gamepad1.x && !reverseFlywheels) {
                shooterActive = false;
                shoot = false;
                reverseFlywheels = true;
                _6000RPMmotor.setVelocity(targetTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(-targetTicksPerSec);
            }

            // --- Manual Intake ---
            if (!shoot) {
                if (gamepad1.right_bumper) {
                    _1150RPMintake.setPower(RobotConfig.INTAKE_MOTOR_POWER_INWARD);
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_REVERSE);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_REVERSE);
                } else if (gamepad1.left_bumper) {
                    _1150RPMintake.setPower(RobotConfig.INTAKE_MOTOR_POWER_OUTWARD);
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                } else {
                    _1150RPMintake.setPower(0);
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                }
            }

            // --- Intelligent Shooting Sequence ---
            if (shoot) {
                // Use a BOOSTED RPM target during active shooting to keep double shots strong
                double boostedRPM = interpolatedRPM + RobotConfig.SHOOTER_BURST_COMPENSATION_RPM;
                double shootTicks = boostedRPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;
                
                _6000RPMmotor.setVelocity(-shootTicks);
                _6000RPMmotorflywheelright.setVelocity(shootTicks);

                // Check if already firing to use wider stall tolerance
                double currentTolerance = (finalIntakeServo.getPower() > 0.1) 
                        ? RobotConfig.SHOOTER_STALL_TOLERANCE 
                        : RobotConfig.SHOOTER_RPM_TOLERANCE;

                // Firing check: use the interpolated target (the baseline) for the initial hit
                boolean rpmAtTarget = Math.abs(leftRPM) >= (interpolatedRPM - currentTolerance);

                if (rpmAtTarget) {
                    shootTimer.reset();
                }

                // Burst fire: Keep feeder running if RPM is good OR within duration since last hit
                if (shootTimer.milliseconds() < RobotConfig.SHOOT_DURATION_MS) {
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_SHOOT);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_SHOOT);
                    _1150RPMintake.setPower(RobotConfig.INTAKE_MOTOR_POWER_INWARD);
                } else {
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    _1150RPMintake.setPower(0);
                }
            } else if (!reverseFlywheels) {
                // Background spooling: maintain the dynamic target based on target visibility
                _6000RPMmotor.setVelocity(-targetTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(targetTicksPerSec);
            }

            // --- Telemetry ---
            telemetry.addData("Status", shoot ? "SHOOTING" : "READY");
            telemetry.addData("Calculated RPM", Math.round(interpolatedRPM));
            telemetry.addData("Actual RPM", Math.round(leftRPM));
            telemetry.update();
        }
    }
}
