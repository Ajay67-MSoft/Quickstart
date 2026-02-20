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
 * TeleOp OpMode for Auto Aiming and Shooting.
 * This code handles robot movement, Limelight-based auto-aiming, 
 * flywheel speed interpolation based on distance, and an automated shooting sequence.
 * 
 * Refactored to use RobotConfig.java for shared constants and tuning.
 */
@TeleOp(name = "AutoAiming ----------------------------------")
public class AutoAiming extends LinearOpMode {

    // --- Hardware Declarations ---
    private CRServo FinalIntakeLeftDS;   // Feeder servo left
    private CRServo finalIntakeServo;    // Feeder servo right

    private DcMotor frontLeftWheelDS;
    private DcMotor backLeftWheelDS;
    private DcMotor frontRightWheelDS;
    private DcMotor backRightWheelDS;

    private DcMotor _1150RPMintake;      // Main intake motor

    private DcMotorEx _6000RPMmotor;     // Left flywheel motor
    private DcMotorEx _6000RPMmotorflywheelright; // Right flywheel motor

    private Limelight3A limelight;       // Vision sensor

    // Drive variables
    private double x, y, rx;

    // State variables
    private boolean shoot = false;           // Is the shooting sequence active?
    private boolean shooterActive = false;   // Is the shooter toggled on?
    private boolean reverseFlywheels = false;// Are flywheels reversing to clear jams?
    private ElapsedTime timer = new ElapsedTime();

    private double TARGET_SHOOT_RPM = 3100;
    private double targetTicksPerSec;

    @Override
    public void runOpMode() {
        // Load external configuration from CSV if present
        RobotConfig.loadLookupTable();

        // --- Hardware Mapping (Linking code to actual robot parts) ---
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

        // Set motor directions for proper chassis movement
        frontRightWheelDS.setDirection(DcMotor.Direction.REVERSE);
        backRightWheelDS.setDirection(DcMotor.Direction.REVERSE);

        // Allow motors to spin freely when power is 0 (prevents jerky stops)
        frontLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightWheelDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.setMsTransmissionInterval(11); // Faster telemetry updates
        limelight.pipelineSwitch(0);             // Use standard vision pipeline
        limelight.start();

        // Initial servo states
        FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
        FinalIntakeLeftDS.setDirection(CRServo.Direction.REVERSE);
        finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);

        // Configure flywheel motors for velocity control
        _6000RPMmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _6000RPMmotorflywheelright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- PIDF for flywheel motors (Advanced "Cruise Control" for speed) ---
        _6000RPMmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, RobotConfig.SHOOTER_PIDF);
        _6000RPMmotorflywheelright.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, RobotConfig.SHOOTER_PIDF);

        waitForStart();

        while (opModeIsActive()) {

            // --- Read Driver Joystick Input ---
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x * 0.75;

            // Get vision data from Limelight
            LLResult result = limelight.getLatestResult();

            // --- Auto-Aim Logic (Hold B Button) ---
            if (gamepad1.b && result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal offset from target
                double absTx = Math.abs(tx);
                double alignedThreshold = RobotConfig.AIM_TOLERANCE_DEGREES; // How many degrees off-center we accept
                double minRx = RobotConfig.AIM_MIN_ROTATION_POWER;
                double maxRx = RobotConfig.AIM_MAX_ROTATION_POWER;

                if (absTx <= alignedThreshold) {
                    rx = 0; // Locked on target
                } else {
                    // Calculate rotation speed: the further away, the faster it turns
                    double scale = (absTx > 2) ? maxRx : Math.pow(absTx / 2.0, 1.5) * maxRx;
                    if (scale < minRx) scale = minRx;
                    rx = -Math.signum(tx) * scale;
                }
            }

            // --- Apply Chassis Movement ---
            frontLeftWheelDS.setPower(y + x + rx);
            backLeftWheelDS.setPower(y - x + rx);
            frontRightWheelDS.setPower(y - x - rx);
            backRightWheelDS.setPower(y + x - rx);

            // --- Target RPM Calculation ---
            // If Limelight sees target, calculate RPM based on distance (TY)
            TARGET_SHOOT_RPM = 3100;
            if (result != null && result.isValid()) TARGET_SHOOT_RPM = RobotConfig.getInterpolatedRPM(result.getTy());
            targetTicksPerSec = TARGET_SHOOT_RPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;

            // Measure current flywheel speeds
            double leftRPM = _6000RPMmotor.getVelocity() * 60.0 / RobotConfig.SHOOTER_TICKS_PER_REV;
            double rightRPM = _6000RPMmotorflywheelright.getVelocity() * 60.0 / RobotConfig.SHOOTER_TICKS_PER_REV;

            // --- Shooter Controls ---
            if (gamepad1.yWasPressed()) {
                shooterActive = true;
                shoot = true;      // Start the shooting sequence
                reverseFlywheels = false;
                timer.reset();
            }

            if (gamepad1.aWasPressed()) {
                shooterActive = false;
                shoot = false;
                reverseFlywheels = false;
                // Idle speed
                double idleTicks = RobotConfig.SHOOTER_IDLE_RPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;
                _6000RPMmotor.setVelocity(-idleTicks);
                _6000RPMmotorflywheelright.setVelocity(idleTicks);
            }

            // --- X Button: Reverse flywheels once to clear jams ---
            if (gamepad1.x && !reverseFlywheels) {
                shooterActive = false;
                shoot = false;
                reverseFlywheels = true;
                _6000RPMmotor.setVelocity(targetTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(-targetTicksPerSec);
            }

            // --- DPad Down: View Limelight Telemetry for debugging ---
            if (gamepad1.dpad_down && result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }

            // --- Manual Intake (Bumpers) ---
            if (!shoot) { // Only allow manual intake if NOT shooting
                if (gamepad1.right_bumper) {
                    _1150RPMintake.setPower(RobotConfig.INTAKE_MOTOR_POWER_INWARD);
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_REVERSE); // Hold ball back
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
                // Step 1: Spin flywheels toward target RPM
                _6000RPMmotor.setVelocity(-targetTicksPerSec);
                _6000RPMmotorflywheelright.setVelocity(targetTicksPerSec);

                // Step 2: Check if flywheels are at the correct speed
                if (Math.abs(leftRPM) >= TARGET_SHOOT_RPM - RobotConfig.SHOOTER_RPM_TOLERANCE
                        && Math.abs(leftRPM) <= TARGET_SHOOT_RPM + RobotConfig.SHOOTER_RPM_TOLERANCE) {

                    // Step 3: Speed is correct → Run feeders to fire the ball
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_SHOOT);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_SHOOT);
                    _1150RPMintake.setPower(RobotConfig.INTAKE_MOTOR_POWER_INWARD);
                } else {
                    // Step 4: Not at speed yet → Keep feeders stopped
                    finalIntakeServo.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    FinalIntakeLeftDS.setPower(RobotConfig.FEEDER_SERVO_POWER_HOLD);
                    _1150RPMintake.setPower(0);
                }
            }

            // --- Flywheel Idle Logic ---
            if (!shoot && !reverseFlywheels && !shooterActive) {
                double idleTicks = RobotConfig.SHOOTER_IDLE_RPM * RobotConfig.SHOOTER_TICKS_PER_REV / 60.0;
                _6000RPMmotor.setVelocity(-idleTicks);
                _6000RPMmotorflywheelright.setVelocity(idleTicks);
            }

            // --- Telemetry Display on Driver Station ---
            telemetry.addData("shoot:", shoot);
            telemetry.addData("Target RPM:", TARGET_SHOOT_RPM);
            telemetry.addData("Left Flywheel RPM:", Math.round(leftRPM));
            telemetry.addData("Right Flywheel RPM:", Math.round(rightRPM));
            telemetry.update();
        }
    }
}
