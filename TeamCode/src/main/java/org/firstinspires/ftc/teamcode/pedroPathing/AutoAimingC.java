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

    private double TARGET_SHOOT_RPM = 3100;
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

        // Set motor directions
        frontRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotor.Direction.REVERSE);

        // Zero power behavior
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

        // Configure flywheel motors
        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply shared PIDF coefficients
        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, RobotConfig.SHOOTER_PIDF);
        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, RobotConfig.SHOOTER_PIDF);

        // Initialize shootTimer to a value larger than SHOOT_DURATION_MS to prevent accidental firing at start
        shootTimer.reset();

        waitForStart();

        while (opModeIsActive()) {

            // --- Driver Input ---
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x * 0.75;

            LLResult result = limelight.getLatestResult();

            // --- Auto-Aim Logic (Hold B) ---
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

            // --- RPM Calculation ---
            TARGET_SHOOT_RPM = 3100;
            if (result != null && result.isValid()) TARGET_SHOOT_RPM = RobotConfig.getInterpolatedRPM(result.getTy());
            targetTicksPerSec = TARGET_SHOOT_RPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;

            double leftRPM = _6000RPMmotor.getVelocity() * 60.0 / RobotConfig.SHOOTER_TICKS_PER_REV;
            double rightRPM = _6000RPMmotorflywheelright.getVelocity() * 60.0 / RobotConfig.SHOOTER_TICKS_PER_REV;

            // --- Controls ---
            if (gamepad1.yWasPressed()) {
                shooterActive = true;
                shoot = true;      // start shooting sequence
                reverseFlywheels = false;
                timer.reset();
            }

            if (gamepad1.aWasPressed()) {
                shooterActive = false;
                shoot = false;
                reverseFlywheels = false;
                double idleTicks = RobotConfig.SHOOTER_IDLE_RPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;
                _6000RPMmotor.setVelocity(-idleTicks);
                _6000RPMmotorflywheelright.setVelocity(idleTicks);
            }

            if (gamepad1.x && !reverseFlywheels) {
                shooterActive = false;
                shoot = false;
                reverseFlywheels = true;
                _6000RPMmotor.setVelocity(targetTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(-targetTicksPerSec);
            }

            if (gamepad1.dpad_down && result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
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

            // --- Shooting Sequence with Minimum Duration (Anti-Oscillation) ---
            if (shoot) {
                _6000RPMmotor.setVelocity(-targetTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(targetTicksPerSec);

                // Check if current RPM is within tolerance
                boolean rpmAtTarget = Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RobotConfig.SHOOTER_RPM_TOLERANCE
                        && Math.abs(leftRPM) <= TARGET_SHOOT_RPM + RobotConfig.SHOOTER_RPM_TOLERANCE;

                // If RPM is at target, reset the timer to keep the feeder active
                if (rpmAtTarget) {
                    shootTimer.reset();
                }

                // Maintain shooting action if we are at target OR if it's been less than SHOOT_DURATION_MS since last hit
                if (shootTimer.milliseconds() < RobotConfig.SHOOT_DURATION_MS) {
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_SHOOT);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_SHOOT);
                    _1150RPMintake.setPower(RobotConfig.INTAKE_MOTOR_POWER_INWARD);
                } else {
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    _1150RPMintake.setPower(0);
                }
            }

            if (!shoot && !reverseFlywheels && !shooterActive) {
                double idleTicks = RobotConfig.SHOOTER_IDLE_RPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;
                _6000RPMmotor.setVelocity(-idleTicks);
                _6000RPMmotorflywheelright.setVelocity(idleTicks);
            }

            // --- Telemetry ---
            telemetry.addData("Status", "AutoAimingC (Shared Config)");
            telemetry.addData("Target RPM", Math.round(TARGET_SHOOT_RPM));
            telemetry.addData("Left RPM", Math.round(leftRPM));
            telemetry.addData("Right RPM", Math.round(rightRPM));
            telemetry.addData("Shooting Active", shoot);
            telemetry.update();
        }
    }
}
