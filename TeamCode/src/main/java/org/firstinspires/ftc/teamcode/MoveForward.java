package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name = "Limemight testing", group = "Samples")
public class MoveForward extends OpMode {

    private DcMotor frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeftWheelDS");
    private DcMotor backLeftWheel = hardwareMap.get(DcMotor.class, "backLeftWheelDS");
    private DcMotor frontRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheelDS");
    private DcMotor backRightWheel = hardwareMap.get(DcMotor.class, "backRightWheelDS");

    private Limelight3A limelight3A;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0); // 0 is blue, 1 is red
 //       DcMotor leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
 //       DcMotor rightMotor = hardwareMap.get(DcMotor.class, "right_motor");


        telemetry.addData("Status", "Ready to move forward!");
        telemetry.update();
//        waitForStart();

        // Step 3: Move forward
        telemetry.addData("Status", "Moving forward...");
        telemetry.update();

   //     leftMotor.setPower(0.5);   // 50% power forward
   //     rightMotor.setPower(0.5);  // 50% power forward

        // Step 4: Wait for 2 seconds
//        time.sleep(2000);  // 2000 milliseconds = 2 seconds

        // Step 5: Stop
   //     leftMotor.setPower(0);
   //     rightMotor.setPower(0);

        telemetry.addData("Status", "Done!");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight3A.start();
        init();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        System.out.println(llResult);

        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area offset", llResult.getTa());
        }
        else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
