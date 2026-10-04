package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "flywheel tester")
//@Disabled
public class flywheelTester extends OpMode {

    // Declare OpMode members.
    private DcMotorEx flywheel = null;

    /* ================= SHOOTER CONSTANTS ================= */
    private static final double TICKS_PER_REV = 28.0;
    private double TARGET_SHOOT_RPM = 2780.0; // adjustable with d-pad, 10 RPM per press
    // bang-bang -> PIDF handoff threshold (RPM below target where we stop
    // full-powering and let the PIDF setVelocity() take over)
//    private static final double HYBRID_THRESHOLD_RPM = TARGET_SHOOT_RPM - 150; // e.g. 2900 for a 3000 target

//    private boolean shooterOn = false;

    private enum FlywheelMode { OFF, IDLE, RAMP, SHOOT }
    private FlywheelMode mode = FlywheelMode.IDLE;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

      /*
        * Initialize the hardware variables. Note that the strings used here
as parameters
        * to 'get' must correspond to the names assigned during the robot
configuration
        * step.
        */

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setVelocityPIDFCoefficients(1, 0.02, 0.005, 12);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

   /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit
START
    * I commented it out so don't worry about that ;-;
    */
   /*
   @Override
   public void init_loop() {
   }
   */


    /*
     * Code to run ONCE when the driver hits START
     * I also commented this out so don't worry about that :D
     */
  /*
  @Override
  public void start() {
  }
  */


    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit
 STOP
     */
    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())  {
            TARGET_SHOOT_RPM += 10;
        }
        if (gamepad1.dpadDownWasPressed()) {
            TARGET_SHOOT_RPM -= 10;
        }

        // Read every button every loop so presses never get "saved up"
        boolean y = gamepad1.yWasPressed();
        boolean a = gamepad1.aWasPressed();
        boolean x = gamepad1.xWasPressed();
        boolean b = gamepad1.bWasPressed();

// Buttons only change the mode
        if (y)      mode = FlywheelMode.SHOOT; // hybrid spin-up to target
        else if (a) mode = FlywheelMode.IDLE;  // back to 0.1 power
        else if (x) mode = FlywheelMode.OFF;   // no power
        else if (b) mode = FlywheelMode.RAMP;  // 0.375 power

// The mode controls the motor every loop, so it stays set
        switch (mode) {
            case SHOOT:
                android.util.Log.d("flywheel mode", System.currentTimeMillis() + "shoot");
                spinUpFlywheelHybrid();
                break;
            case RAMP:
                android.util.Log.d("flywheel mode", System.currentTimeMillis() + "ramp up");
                flywheel.setPower(0.375);
                break;
            case OFF:
                android.util.Log.d("flywheel mode", System.currentTimeMillis() + "off");
                flywheel.setPower(0);
                break;
            case IDLE:
            default:
                android.util.Log.d("flywheel mode", System.currentTimeMillis() + "idle");
                flywheel.setPower(0.1);
                break;
        }

        telemetry.addData("Flywheel Mode", mode);

        double currentRPM = flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        telemetry.addData("Current Flywheel RPM", Math.abs(currentRPM));
        telemetry.addData("Target RPM", TARGET_SHOOT_RPM);
        telemetry.addData("Hybrid Threshold RPM", TARGET_SHOOT_RPM - 150);

        android.util.Log.d("RPM_LOG", System.currentTimeMillis() + "," +
                flywheel.getVelocity());

        if (gamepad1.right_trigger_pressed) {
            android.util.Log.d("POLLEN_SHOT", System.currentTimeMillis() + "a pollen was just shot!");
        }

    }

    private void spinUpFlywheelHybrid() {
//        double currentRPM = Math.abs(flywheel.getVelocity() * 60.0 / TICKS_PER_REV);
        double targetTicksPerSec = TARGET_SHOOT_RPM * TICKS_PER_REV / 60.0;

//        if (currentRPM < TARGET_SHOOT_RPM - 150) {
//            flywheel.setPower(1.0); // full power until close to target
//        } else {
        flywheel.setVelocity(targetTicksPerSec); // PIDF takes over
//        }
    }
}