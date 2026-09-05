//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import android.util.Size;
//import org.opencv.imgproc.Imgproc;
//
//// ============================================
//// UTILITY CLASS (for use in any OpMode)
//// ============================================
//class BallDetector {
//    private VisionPortal visionPortal;
//    private BallProcessor ballProcessor;
//
//    public BallDetector(HardwareMap hardwareMap) {
//        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        ballProcessor = new BallProcessor();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(webcam)
//                .addProcessor(ballProcessor)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(false)
//                .build();
//    }
//
//    public boolean isBallDetected() {
//        return ballProcessor.hasAnyBall();
//    }
//
//    public String getDetectedBallColor() {
//        return ballProcessor.getDetectedColor();
//    }
//
//    public boolean quickScanForBalls() {
//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
//        return isBallDetected();
//    }
//
//    public void printTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
//        telemetry.addData("Ball Detected", isBallDetected() ? "YES" : "NO");
//        telemetry.addData("Ball Color", getDetectedBallColor());
//    }
//
//    public void close() {
//        if (visionPortal != null) {
//            visionPortal.close();
//        }
//    }
//
//    // ============================================
//    // INTERNAL PROCESSOR CLASS
//    // ============================================
//    private static class BallProcessor implements VisionProcessor {
//        @Override
//        public void init(int width, int height, CameraCalibration calibration) {
//            // Remove everything from this method or just leave it empty
//        }
//        private Mat hsv = new Mat();
//        private Mat mask = new Mat();
//
//        // HSV ranges for CENTERSTAGE pixels
//        private final Scalar[] LOWER_BOUNDS = {
//                new Scalar(20, 100, 100),   // Yellow
//                new Scalar(0, 0, 200),      // White
//                new Scalar(40, 100, 100),   // Green
//                new Scalar(120, 100, 100)   // Purple
//        };
//
//        private final Scalar[] UPPER_BOUNDS = {
//                new Scalar(30, 255, 255),   // Yellow
//                new Scalar(180, 30, 255),   // White
//                new Scalar(80, 255, 255),   // Green
//                new Scalar(160, 255, 255)   // Purple
//        };
//
//        private final String[] COLOR_NAMES = {"Yellow", "White", "Green", "Purple"};
//
//        private boolean ballFound = false;
//        private String detectedColor = "None";
//        private int lastPixelCount = 0;
//
//        @Override
//        public Object processFrame(Mat frame, long captureTimeNanos) {
//            ballFound = false;
//            detectedColor = "None";
//
//            if (frame.empty()) return frame;
//
//            // Convert to HSV
//            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
//
//            // Focus on bottom half (where balls are)
//            int height = hsv.height();
//            int width = hsv.width();
//            Rect groundROI = new Rect(0, height/2, width, height/2);
//            Mat groundRegion = new Mat(hsv, groundROI);
//
//            // Check each color
//            for (int i = 0; i < COLOR_NAMES.length; i++) {
//                Core.inRange(groundRegion, LOWER_BOUNDS[i], UPPER_BOUNDS[i], mask);
//                int pixelCount = Core.countNonZero(mask);
//                lastPixelCount = pixelCount;
//
//                if (pixelCount > 300) {  // Detection threshold
//                    ballFound = true;
//                    detectedColor = COLOR_NAMES[i];
//                    break;
//                }
//            }
//
//            return frame;
//        }
//
//        @Override
//        public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
//                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
//                                Object userContext) {
//            // Optional drawing
//        }
//
//        public boolean hasAnyBall() {
//            return ballFound;
//        }
//
//        public String getDetectedColor() {
//            return detectedColor;
//        }
//
//        public int getLastPixelCount() {
//            return lastPixelCount;
//        }
//    }
//}
//
//// ============================================
//// OP-MODE 1: Simple Test OpMode
//// ============================================
//@TeleOp(name = "Ball Detection Test", group = "Test")
//class BallDetectionTest extends LinearOpMode {
//    private BallDetector ballDetector;
//
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initializing...");
//        telemetry.update();
//
//        // Create detector
//        ballDetector = new BallDetector(hardwareMap);
//
//        // Wait for camera
//        sleep(1000);
//
//        telemetry.addData("Status", "Ready! Press Start");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Get detection status
//            boolean ballFound = ballDetector.isBallDetected();
//            String color = ballDetector.getDetectedBallColor();
//
//            // Display telemetry
//            telemetry.clear();
//            telemetry.addLine("=== BALL DETECTION ===");
//            telemetry.addData("Ball Found", ballFound ? "🎯 YES" : "❌ NO");
//            telemetry.addData("Color", color);
//            telemetry.addLine("");
//            telemetry.addLine("Controls:");
//            telemetry.addData("A", "Quick scan");
//            telemetry.addData("B", "Print details");
//            telemetry.update();
//
//            // Controls
//            if (gamepad1.a) {
//                telemetry.addLine("Quick scanning...");
//                telemetry.update();
//                sleep(100);
//                telemetry.addData("Result", ballDetector.quickScanForBalls() ? "Found!" : "Not found");
//                telemetry.update();
//                sleep(1000);
//            }
//
//            if (gamepad1.b) {
//                ballDetector.printTelemetry(telemetry);
//                telemetry.update();
//                sleep(500);
//            }
//
//            sleep(50);
//        }
//
//        // Clean up
//        ballDetector.close();
//    }
//}
//
//// ============================================
//// OP-MODE 2: TeleOp with Ball Detection
//// ============================================
//@TeleOp(name = "Drive with Ball Detection", group = "Competition")
//class DriveWithBallDetection extends LinearOpMode {
//    private BallDetector ballDetector;
//    private com.qualcomm.robotcore.hardware.DcMotor leftFront, rightFront, leftBack, rightBack;
//    private com.qualcomm.robotcore.hardware.DcMotor intakeMotor;
//
//    @Override
//    public void runOpMode() {
//        // Initialize motors
//        leftFront = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "leftFront");
//        rightFront = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "rightFront");
//        leftBack = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "leftBack");
//        rightBack = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "rightBack");
//        intakeMotor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "intakeMotor");
//
//        // Initialize detector
//        ballDetector = new BallDetector(hardwareMap);
//
//        telemetry.addData("Status", "Ready to drive");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // ====================
//            // DRIVE CONTROLS
//            // ====================
//            double drive = -gamepad1.left_stick_y;
//            double strafe = gamepad1.left_stick_x;
//            double turn = gamepad1.right_stick_x;
//
//            // Mecanum calculations
//            double leftFrontPower = drive + strafe + turn;
//            double rightFrontPower = drive - strafe - turn;
//            double leftBackPower = drive - strafe + turn;
//            double rightBackPower = drive + strafe - turn;
//
//            // Normalize
//            double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
//                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
//            if (maxPower > 1.0) {
//                leftFrontPower /= maxPower;
//                rightFrontPower /= maxPower;
//                leftBackPower /= maxPower;
//                rightBackPower /= maxPower;
//            }
//
//            // Set powers
//            leftFront.setPower(leftFrontPower);
//            rightFront.setPower(rightFrontPower);
//            leftBack.setPower(leftBackPower);
//            rightBack.setPower(rightBackPower);
//
//            // ====================
//            // INTAKE CONTROLS
//            // ====================
//            if (gamepad1.right_trigger > 0.1) {
//                intakeMotor.setPower(gamepad1.right_trigger);
//            } else if (gamepad1.left_trigger > 0.1) {
//                intakeMotor.setPower(-gamepad1.left_trigger);
//            } else {
//                intakeMotor.setPower(0);
//            }
//
//            // ====================
//            // BALL DETECTION
//            // ====================
//            boolean ballFound = ballDetector.isBallDetected();
//            String color = ballDetector.getDetectedBallColor();
//
//            // Auto-pickup when X pressed and ball detected
//            if (gamepad1.x && ballFound) {
//                autoPickup();
//            }
//
//            // ====================
//            // TELEMETRY
//            // ====================
//            telemetry.clear();
//            telemetry.addLine("=== DRIVE STATUS ===");
//            telemetry.addData("Drive Power", "%.2f", drive);
//            telemetry.addData("Strafe", "%.2f", strafe);
//            telemetry.addData("Turn", "%.2f", turn);
//            telemetry.addLine("");
//
//            telemetry.addLine("=== BALL DETECTION ===");
//            telemetry.addData("Ball Found", ballFound ? "🎯 YES - " + color : "❌ NO");
//            telemetry.addLine("");
//
//            telemetry.addLine("=== CONTROLS ===");
//            telemetry.addData("X + Ball", "Auto pickup");
//            telemetry.addData("RT/LT", "Intake in/out");
//            telemetry.update();
//
//            sleep(20);
//        }
//
//        ballDetector.close();
//    }
//
//    private void autoPickup() {
//        telemetry.addLine("Starting auto pickup...");
//        telemetry.update();
//
//        // Stop driving
//        setMotorPowers(0, 0, 0, 0);
//
//        // Approach slowly
//        setMotorPowers(0.2, 0.2, 0.2, 0.2);
//        sleep(600);
//
//        // Run intake
//        intakeMotor.setPower(0.8);
//        sleep(800);
//
//        // Back up
//        intakeMotor.setPower(0);
//        setMotorPowers(-0.15, -0.15, -0.15, -0.15);
//        sleep(300);
//
//        // Stop
//        setMotorPowers(0, 0, 0, 0);
//
//        telemetry.addLine("Pickup complete!");
//        telemetry.update();
//        sleep(500);
//    }
//
//    private void setMotorPowers(double lf, double rf, double lb, double rb) {
//        leftFront.setPower(lf);
//        rightFront.setPower(rf);
//        leftBack.setPower(lb);
//        rightBack.setPower(rb);
//    }
//}
//
//// ============================================
//// OP-MODE 3: Autonomous Ball Finder
//// ============================================
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto Ball Finder", group = "Auto")
//class AutoBallFinder extends LinearOpMode {
//    private BallDetector ballDetector;
//    private com.qualcomm.robotcore.hardware.DcMotor leftMotor, rightMotor;
//    private com.qualcomm.robotcore.hardware.DcMotor intakeMotor;
//
//    @Override
//    public void runOpMode() {
//        // Initialize hardware
//        leftMotor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "leftMotor");
//        rightMotor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "rightMotor");
//        intakeMotor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "intakeMotor");
//
//        // Set motor directions
//        leftMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
//
//        // Initialize detector
//        ballDetector = new BallDetector(hardwareMap);
//
//        telemetry.addData("Status", "Ready for Autonomous");
//        telemetry.update();
//
//        waitForStart();
//
//        // ====================
//        // STEP 1: SCAN FOR BALL
//        // ====================
//        telemetry.addLine("Step 1: Scanning for ball...");
//        telemetry.update();
//
//        boolean ballFound = scanForBall();
//
//        if (ballFound) {
//            // ====================
//            // STEP 2: APPROACH BALL
//            // ====================
//            telemetry.addLine("Step 2: Approaching ball...");
//            telemetry.update();
//
//            approachBall();
//
//            // ====================
//            // STEP 3: PICK UP BALL
//            // ====================
//            telemetry.addLine("Step 3: Picking up ball...");
//            telemetry.update();
//
//            pickUpBall();
//
//            // ====================
//            // STEP 4: GO TO DEPOSIT
//            // ====================
//            telemetry.addLine("Step 4: Going to deposit...");
//            telemetry.update();
//
//            goToDeposit();
//
//            telemetry.addLine("Autonomous Complete!");
//        } else {
//            telemetry.addLine("No ball found. Going to backup position.");
//            goToBackupPosition();
//        }
//
//        telemetry.update();
//        ballDetector.close();
//        sleep(2000);
//    }
//
//    private boolean scanForBall() {
//        // Turn slowly while scanning
//        for (int i = 0; i < 12; i++) {  // Scan for ~6 seconds
//            if (ballDetector.isBallDetected()) {
//                telemetry.addLine("Ball found!");
//                return true;
//            }
//
//            // Turn 30 degrees
//            leftMotor.setPower(0.2);
//            rightMotor.setPower(-0.2);
//            sleep(500);
//
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//            sleep(100);  // Check again
//        }
//        return false;
//    }
//
//    private void approachBall() {
//        // Drive forward until ball is close (simplified)
//        for (int i = 0; i < 5; i++) {
//            leftMotor.setPower(0.25);
//            rightMotor.setPower(0.25);
//            sleep(400);
//
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//
//            if (ballDetector.isBallDetected()) {
//                sleep(100);
//            } else {
//                break;  // Lost ball or too close
//            }
//        }
//    }
//
//    private void pickUpBall() {
//        // Run intake
//        intakeMotor.setPower(0.8);
//        sleep(1000);
//
//        // Back up a little
//        intakeMotor.setPower(0);
//        leftMotor.setPower(-0.2);
//        rightMotor.setPower(-0.2);
//        sleep(300);
//
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//
//    private void goToDeposit() {
//        // Turn 180 degrees
//        leftMotor.setPower(0.3);
//        rightMotor.setPower(-0.3);
//        sleep(1000);
//
//        // Drive forward to deposit
//        leftMotor.setPower(0.3);
//        rightMotor.setPower(0.3);
//        sleep(1500);
//
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//
//    private void goToBackupPosition() {
//        // Drive to a safe position
//        leftMotor.setPower(0.3);
//        rightMotor.setPower(0.3);
//        sleep(1000);
//
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//}