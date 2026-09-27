package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "StarterBot Teleop", group = "StarterBot")
//@Disabled
public class starterBotTeleop extends OpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    private CRServo indexer = null;
    private DcMotorEx flywheel = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    // Set up a variable for each drive wheel to save power level for telemetry.
    double leftPower;
    double rightPower;

    // Create a variable to set to the intake.
    double intakePower;

    /* ================= SHOOTER CONSTANTS ================= */
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_SHOOT_RPM = 3000.0; // change this one value to change shot speed
    // bang-bang -> PIDF handoff threshold (RPM below target where we stop
    // full-powering and let the PIDF setVelocity() take over)
    private static final double HYBRID_THRESHOLD_RPM = TARGET_SHOOT_RPM - 100; // e.g. 2900 for a 3000 target

    private boolean shooterOn = false;

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

        // the thing to the left of the = sign is what's used in this code
        // the thing on the far right in "" is what's used in the DS
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        indexer = hardwareMap.get(CRServo.class, "indexer");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class,
                "right_intake_servo");

        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel.setVelocityPIDFCoefficients(1, 0.02, 0.005, 12);

       /*
        * To drive forward, most robots need the motor on one side to be
reversed,
        * because the axles point in opposite directions. Pushing the left
stick forward
        * MUST make robot go forward. So adjust these two lines based on your
first test drive.
        * Note: The settings here assume direct drive on left and right
wheels. Gear
        * Reduction or 90 Deg drives may require direction flips
        */
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

       /*
        * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This
causes the motor to
        * slow down much faster when it is coasting. This creates a much more
controllable
        * drivetrain. As the robot stops much quicker.
        */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

       /*
        * Much like our drivetrain motors, we set the right intake servo to
reverse so that both
        * servos work to pull elements into the intake.
        */
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

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
       /*
        * Here we call a function called arcadeDrive. The arcadeDrive function
takes the input from
        * the joysticks, and applies power to the left and right drive motor
to move the robot
        * as requested by the driver. "arcade" refers to the control style
we're using here.
        * Much like a classic arcade game, when you move the left joystick
forward both motors
        * work to drive the robot forward, and when you move the right
joystick left and right
        * both motors work to rotate the robot. Combinations of these inputs
can be used to create
        * more complex maneuvers.
        */
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

       /*
        * Set the intake power variable to equal the right trigger, minus the
left trigger.
        * Each trigger outputs a signal from 0-1, with 0 as fully released,
and 1 fully depressed.
        * This gives us proportional control of the intake speed. The speed
increases as we pull
        * the right trigger further. It's occasionally helpful to be able to
reverse the intake,
        * so we also factor in the the left trigger. If the left trigger is
fully depressed,
        * the intakePower variable will be -1. If the right trigger is fully
depressed, the variable
        * will be 1. If the driver pulls both triggers, the intake will remain
off.
        * We use this technique (creating a variable, and setting it to our
control inputs) to
        * allow us to avoid setting the same motors/servos power more than
once per loop. That can
        * create erratic behavior.
        */

        if (gamepad1.right_trigger_pressed) {
            rightIntakeServo.setPower(1);
        } else {
            rightIntakeServo.setPower(0);
        }
        if (gamepad1.left_trigger_pressed) {
            leftIntakeServo.setPower(1);
        } else {
            leftIntakeServo.setPower(0);
        }

        if (gamepad1.right_bumper) { // changed from right trigger to right bumper
                    intakePower = 1;
        }
        else if (gamepad1.left_bumper) {
            intakePower = -1;
        }
        else {
            intakePower = 0;
        }

        intake.setPower(intakePower);
//          leftIntakeServo.setPower(intakePower);
//          rightIntakeServo.setPower(intakePower);

        /*
         * Show motor powers on the Driver Station via telemetry.
         * Commented out because it costs extra ms during loop
         * times, though that probably won't matter much since
         * we're not using this in competition any time soon.
         */
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//        telemetry.addData("Triggers", "left (%.2f, right (%.2f)",gamepad1.left_trigger, gamepad1.right_trigger);

        if (gamepad1.yWasPressed()) {
            shooterOn = true;
            indexer.setPower(1);
        }
        else if (gamepad1.aWasPressed()) {
            shooterOn = false;
            indexer.setPower(0);
        }

        // Flywheel is driven every loop while shooterOn is true, so the
        // bang-bang -> PIDF handoff can actually react to current speed
        // instead of firing once on the button edge.
        if (shooterOn) {
            spinUpFlywheelHybrid();
        } else {
            flywheel.setPower(0); // if we add inertia to the flywheel we can comment this out i think
        }

        double currentRPM = flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        telemetry.addData("Current Flywheel RPM", Math.abs(currentRPM));
        telemetry.addData("Target RPM", TARGET_SHOOT_RPM);
        telemetry.addData("Hybrid Threshold RPM", HYBRID_THRESHOLD_RPM);

        android.util.Log.d("RPM_LOG", System.currentTimeMillis() + "," +
                flywheel.getVelocity());

    }

    /*
     * Bang-bang below HYBRID_THRESHOLD_RPM, PIDF setVelocity() once close to
     * target. Keeps the target-RPM-to-ticks/sec math tied to
     * TARGET_SHOOT_RPM so changing that one constant updates both the
     * handoff point and the PIDF setpoint together.
     */
    private void spinUpFlywheelHybrid() {
        double currentRPM = Math.abs(flywheel.getVelocity() * 60.0 / TICKS_PER_REV);
        double targetTicksPerSec = TARGET_SHOOT_RPM * TICKS_PER_REV / 60.0;

        if (currentRPM < HYBRID_THRESHOLD_RPM) {
            flywheel.setPower(1.0); // full power until close to target
        } else {
            flywheel.setVelocity(targetTicksPerSec); // PIDF takes over
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}