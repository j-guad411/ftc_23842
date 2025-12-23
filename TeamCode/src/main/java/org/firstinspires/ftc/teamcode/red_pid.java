package org.firstinspires.ftc.teamcode;

// Import necessary FTC SDK classes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "red_backpidf")
//@Disabled
public class red_pid extends LinearOpMode {

    // --- Unit Constants ---
    private static final DistanceUnit ODOMETRY_DISTANCE_UNIT = DistanceUnit.MM;
    private static final AngleUnit ODOMETRY_ANGLE_UNIT = AngleUnit.RADIANS;

    // --- Hardware Declarations ---
    private final String FRONT_LEFT_CONFIG_NAME = "left_front_drive";
    private final String BACK_LEFT_CONFIG_NAME = "left_rear_drive";
    private final String FRONT_RIGHT_CONFIG_NAME = "right_front_drive";
    private final String BACK_RIGHT_CONFIG_NAME = "right_rear_drive";
    private final String PINPOINT_CONFIG_NAME = "odo";

    private final String INTAKE_CONFIG_NAME = "intake";
    private final String MIDDLE_CONFIG_NAME = "middle";
    private final String TRANSFER_CONFIG_NAME = "transfer";


    public double highVelocity = 1210;

    public double lowvelocity = 900;

    private final double X_OFFSET_TO_Y_DEADWHEEL_MM = 80.0; //see gobilda pinpoint user manual
    private final double Y_OFFSET_TO_X_DEADWHEEL_MM = -384.0;
    //FTC decode field coordinate definition
    // The DECODE field uses an "inverted" red/blue alliance area configuration,
    // meaning the positive Y-axis points from the Red Wall towNAME = "left shoot";
    //    private final String TRANSFER_CONFIG_NAME="transfer";
    //    private final String MIDDLE_CONFIG_NAME="middle";
    //    private final String INTAKE_CONFIG_NAME="intake";
    //
    //    private final double X_OFFSET_TO_Y_DEADWHEEL_MM = 80.0; //see gobilda pinpoint user manual
    //    private final double Y_OFFSET_TO_X_DEADWHEEL_MM = -384.0; //see gobilda pinpoint user manualard the Blue Alliance,
    // the positive X-axis points toward the audience.
    // The field's origin (0,0,0) is at the centerStarting point in field coordinates
    // The field coordinates for the joystick are currently....
    // Forward on left joystick is +X motion
    // Left on left joystick is +Y motion
    //
    // robot based coordinates
    // +X and 0 deg is points out the front
    // +Y point to the left and is 90 deg
    private final double START_X_VALUE = 0.0;   //17.0 * 25.4 from edge of arena to origin point on robot (in mm);
    private final double START_Y_VALUE = 0.0;
    private final double START_HEADING_VALUE = ODOMETRY_ANGLE_UNIT.fromDegrees(0.0);


    // --- PID Constants ---
    private final double P_DRIVE_COEFF = 0.005;
    private final double I_DRIVE_COEFF = 0.002;
    private final double D_DRIVE_COEFF = 0.0009;
    private final double DRIVE_PID_OUTPUT_LIMIT = 1.0;
    private final double MIN_POWER_TO_MOVE = 0.15;
    private final double POWER_EPSILON = 0.01; // any power command less than this is considered zero

    private final double P_HEADING_COEFF = 0.8;
    private final double I_HEADING_COEFF = 0.00;
    private final double D_HEADING_COEFF = 0.00;
    // --- Tolerances ---
    private final double DISTANCE_TOLERANCE_STOPPED = 25.0;
    private final double DISTANCE_TOLERANCE_PASSING = 100.0;
    private final double HEADING_TOLERANCE_STOPPED = ODOMETRY_ANGLE_UNIT.fromDegrees(2.0);
    private final double HEADING_TOLERANCE_PASSING = ODOMETRY_ANGLE_UNIT.fromDegrees(15.0);
    // --- PID Controllers ---
    private PIDController xPidController;
    private PIDController yPidController;
    private PIDController headingPidController;
    // --- Odometry Variables ---
    private double robotX; // Current X position in ODOMETRY_DISTANCE_UNIT
    private double robotY; // Current Y position in ODOMETRY_DISTANCE_UNIT
    private double robotHeading; // Current heading in ODOMETRY_ANGLE_UNIT (RADIANS, normalized)
    double curTargetVelocity = highVelocity;


    double F = 10;

    double P = 220;

    // --- Hardware ---
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private GoBildaPinpointDriver pinpointOdometry;
    public DcMotorEx leftshoot;
    public DcMotorEx rightshoot;
    private CRServo transfer;
    private CRServo middle;
    private DcMotor intake;
    private RobotState currentState = RobotState.AUTONOMOUS_SEQUENCE_RUNNING;
    private int autonomousSequenceStep = 0;
    private boolean aButtonPreviouslyPressed = false;
    private boolean bButtonPreviouslyPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeDriveMotors();
        initializeGoBildaPinpoint();
        initializePIDs();
        leftshoot = hardwareMap.get(DcMotorEx.class, "left shoot");
        rightshoot = hardwareMap.get(DcMotorEx.class, "rightshoot");
        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftshoot.setDirection(DcMotor.Direction.REVERSE);
        rightshoot.setDirection(DcMotor.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addData("Status", "Initialized. Press A for Auto Sequence, B for Manual.");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {  //while loop after init nut before start
            if (pinpointOdometry != null && pinpointOdometry.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
                updateOdometryVariablesFromPinpoint(); // Keep odometry updated for pre-start display if needed
            }
            displayTelemetry();
            idle();
        }

        if (isStopRequested()) return;

        waitForStart();  //wait for start button to be pressed
        currentState = RobotState.AUTONOMOUS_SEQUENCE_RUNNING; // Start in AUTO

        while (opModeIsActive()) {  //loop waiting for start button to be pressed
            if (!updateOdometryVariablesFromPinpoint()) {
                telemetry.addData("Error", "Pinpoint Driver not available!");
                telemetry.update();
                stopRobot();
                continue;
            }

            boolean aButtonPressed = gamepad1.a;
            boolean bButtonPressed = gamepad1.b;

            // State switching logic
            if (aButtonPressed && !aButtonPreviouslyPressed) {
                if (currentState != RobotState.AUTONOMOUS_SEQUENCE_RUNNING) {
                    currentState = RobotState.AUTONOMOUS_SEQUENCE_RUNNING;
                    autonomousSequenceStep = 0;
                    telemetry.addData("Mode", "Starting Autonomous Sequence");
                }
            }
            aButtonPreviouslyPressed = aButtonPressed;

            if (bButtonPressed && !bButtonPreviouslyPressed) {
                if (currentState != RobotState.MANUAL_DRIVE) {
                    currentState = RobotState.MANUAL_DRIVE;
                    stopRobot();
                    telemetry.addData("Mode", "Switching to Manual Drive");
                }
            }
            bButtonPreviouslyPressed = bButtonPressed;


            // Execute behavior based on current state
            switch (currentState) {
                case IDLE:
                    stopRobot();
                    telemetry.addLine("Status: IDLE. Press A for Auto Sequence, B for Manual Drive.");
                    break;
                case MANUAL_DRIVE:
                    handleManualDrive();
                    telemetry.addLine("Status: MANUAL DRIVE");
                    // Position telemetry is already part of displayTelemetry() which is called after the switch
                    break;
                case AUTONOMOUS_SEQUENCE_RUNNING:
                    telemetry.addLine("Status: AUTONOMOUS SEQUENCE");
                    runAutonomousSequence();
                    // Position and target telemetry are handled within runAutonomousSequence/navigateToTargetWaypoint
                    // and also by the general displayTelemetry()
                    break;
            }

            displayTelemetry(); // This will show general telemetry including position after each loop iteration
            idle();
        }
        stopRobot();
    }

    private void runAutonomousSequence() {
        if (!opModeIsActive()) return; // Exit if OpMode is no longer active

        boolean sequenceFinished = false;
        switch (autonomousSequenceStep) {
            // All Waypoint headings MUST be in RADIANS
            case 0:
                telemetry.addLine("Sequence: Step 1 ( 0, 0.0, 0deg)");
                rightshoot.setVelocity(curTargetVelocity);
                leftshoot.setVelocity(highVelocity);
                rightshoot.setVelocity(highVelocity);
                navigateToTargetWaypoint(new Waypoint(200, -130, ODOMETRY_ANGLE_UNIT.fromDegrees(-23), true));



                transfer.setPower(1);
                middle.setPower(1);
                intake.setPower(1);


                sleep(5500);

                transfer.setPower(0);
                middle.setPower(0);
                intake.setPower(0);

                if (opModeIsActive())
                    autonomousSequenceStep++; // 50 forces end of auto after one move
                break;
            case 1:
                telemetry.addLine("Sequence: Step 2 (0 ft, 0, 90deg)");
                navigateToTargetWaypoint(new Waypoint(250, -400, ODOMETRY_ANGLE_UNIT.fromDegrees(-90), false));
                if (opModeIsActive()) autonomousSequenceStep++;
                break;
            case 2:
                telemetry.addLine("Sequence: Step 3 (0.0, 200, 0deg)");
                intake.setPower(1);
                middle.setPower(1);
                transfer.setPower(1);
                navigateToTargetWaypoint(new Waypoint(250.0, -1200, ODOMETRY_ANGLE_UNIT.fromDegrees(-90), false));

                if (opModeIsActive()) autonomousSequenceStep++;
                break;
            case 3:
                navigateToTargetWaypoint(new Waypoint(200, -700, ODOMETRY_ANGLE_UNIT.fromDegrees(-24), false));
                intake.setPower(0);
                middle.setPower(1);
                transfer.setPower(0);
            case 4:
                telemetry.addLine("Sequence: Step 4 (0, 0, 270deg)");

                navigateToTargetWaypoint(new Waypoint(200, -220.0, ODOMETRY_ANGLE_UNIT.fromDegrees(-23), true));
                leftshoot.setVelocity(highVelocity);
                rightshoot.setVelocity(highVelocity);

                transfer.setPower(1);
                middle.setPower(1);
                intake.setPower(1);


                sleep(5000);

                transfer.setPower(0);
                middle.setPower(0);
                intake.setPower(0);
                navigateToTargetWaypoint(new Waypoint(800, -220.0, ODOMETRY_ANGLE_UNIT.fromDegrees(-90), false));
            case 5:
                transfer.setPower(.55);
                middle.setPower(1);
                intake.setPower(1);

                navigateToTargetWaypoint(new Waypoint(800, -1230.0, ODOMETRY_ANGLE_UNIT.fromDegrees(-90),false));


            case 6:
                leftshoot.setVelocity(lowvelocity);
                rightshoot.setVelocity(lowvelocity);
                navigateToTargetWaypoint(new Waypoint(800, -800, ODOMETRY_ANGLE_UNIT.fromDegrees(-45.0),false));
                navigateToTargetWaypoint(new Waypoint(1503, -403, ODOMETRY_ANGLE_UNIT.fromDegrees(-45.0),false));
                transfer.setPower(0);
                middle.setPower(0);
                intake.setPower(0);
                navigateToTargetWaypoint(new Waypoint(1859, -301, ODOMETRY_ANGLE_UNIT.fromDegrees(-45),true));
                sleep(500);
                transfer.setPower(1);
                middle.setPower(1);
                intake.setPower(1);
                sleep(4900);
                navigateToTargetWaypoint(new Waypoint(1400, -403, ODOMETRY_ANGLE_UNIT.fromDegrees(-45.0),false));
                default:
                telemetry.addLine("Sequence: Finished!");
                sequenceFinished = true;
                break;
        }

        if (sequenceFinished || !opModeIsActive()) {
            stopRobot();
            currentState = RobotState.IDLE; // Return to IDLE when sequence is done or OpMode stops
            autonomousSequenceStep = 0;     // Reset for next time
        }
    }

    private void navigateToTargetWaypoint(Waypoint target) {
        if (!opModeIsActive()) return;


        xPidController.reset();
        yPidController.reset();
        headingPidController.reset();

        ElapsedTime timeoutTimer = new ElapsedTime();
        double NAVIGATION_TIMEOUT_SECONDS = 15.0; // Adjust as needed
        double lastLoopmSec = 0;

        while (opModeIsActive() && !isAtWaypoint(target) && timeoutTimer.seconds() < NAVIGATION_TIMEOUT_SECONDS) {
            if (!updateOdometryVariablesFromPinpoint()) {
                telemetry.addData("Error", "Pinpoint lost during navigation!");
                stopRobot();
                return; // Exit navigation if odometry is lost
            }

            double errorX = target.x - this.robotX; // Field X error
            double errorY = target.y - this.robotY; // Field Y error
            double errorHeading = normalizeAngle(target.heading - this.robotHeading); // Normalized heading error (RADIANS)

            double currentBotHeadingRadians = this.robotHeading; // This is already normalized RADIANS

            // YOUR ORIGINAL Robot-centric error calculations
            // Positive robotCentricErrorX means robot needs to move to its right (robot's local X)
            // Positive robotCentricErrorY means robot needs to move forward (robot's local Y)
            double robotCentricErrorX = errorX * Math.cos(currentBotHeadingRadians) - errorY * Math.sin(-currentBotHeadingRadians);
            double robotCentricErrorY = errorX * Math.sin(currentBotHeadingRadians) - errorY * Math.cos(-currentBotHeadingRadians);

            // YOUR ORIGINAL PID calls and their interpretation
            // xPowerRaw is intended to correct robotCentricErrorX
            // yPowerRaw is intended to correct robotCentricErrorY
            double xPowerRaw = xPidController.calculate(0, -robotCentricErrorX);
            double yPowerRaw = yPidController.calculate(0, -robotCentricErrorY);
            double headingPowerRaw = headingPidController.calculate(0, errorHeading); // Your original sign for heading PID input


            // YOUR ORIGINAL Mecanum wheel power mixing logic from navigateToWaypoint
            double frontLeftPowerUnclipped = xPowerRaw + yPowerRaw + headingPowerRaw;
            double backLeftPowerUnclipped = xPowerRaw - yPowerRaw + headingPowerRaw;
            double frontRightPowerUnclipped = xPowerRaw - yPowerRaw - headingPowerRaw;
            double backRightPowerUnclipped = xPowerRaw + yPowerRaw - headingPowerRaw;

            // Power Scaling Logic (inspired by your original navigateToWaypoint, refined)
            double maxUnclipped = Math.max(Math.abs(frontLeftPowerUnclipped), Math.max(Math.abs(backLeftPowerUnclipped),
                    Math.max(Math.abs(frontRightPowerUnclipped), Math.abs(backRightPowerUnclipped))));

            double scaleFactor;
            if (maxUnclipped < POWER_EPSILON) { // Avoid division by zero if all powers are tiny
                scaleFactor = 0.0;
            } else if (maxUnclipped > DRIVE_PID_OUTPUT_LIMIT) {
                scaleFactor = DRIVE_PID_OUTPUT_LIMIT / maxUnclipped; // Scale down if exceeding limit
            } else if (maxUnclipped < MIN_POWER_TO_MOVE && maxUnclipped > POWER_EPSILON) {
                // Scale up to MIN_POWER_TO_MOVE only if it's currently below it and not zero
                double tempScaleFactor = MIN_POWER_TO_MOVE / maxUnclipped;
                // Ensure scaling up doesn't exceed DRIVE_PID_OUTPUT_LIMIT
                if (maxUnclipped * tempScaleFactor > DRIVE_PID_OUTPUT_LIMIT) {
                    scaleFactor = DRIVE_PID_OUTPUT_LIMIT / maxUnclipped;
                } else {
                    scaleFactor = tempScaleFactor;
                }
            } else {
                scaleFactor = 1.0; // Already in a good range or zero
            }

            double fl = frontLeftPowerUnclipped * scaleFactor;
            double bl = backLeftPowerUnclipped * scaleFactor;
            double fr = frontRightPowerUnclipped * scaleFactor;
            double br = backRightPowerUnclipped * scaleFactor;

            // Final clamp (safety net, though scaling should mostly handle it)
            fl = Math.max(-DRIVE_PID_OUTPUT_LIMIT, Math.min(fl, DRIVE_PID_OUTPUT_LIMIT));
            bl = Math.max(-DRIVE_PID_OUTPUT_LIMIT, Math.min(bl, DRIVE_PID_OUTPUT_LIMIT));
            fr = Math.max(-DRIVE_PID_OUTPUT_LIMIT, Math.min(fr, DRIVE_PID_OUTPUT_LIMIT));
            br = Math.max(-DRIVE_PID_OUTPUT_LIMIT, Math.min(br, DRIVE_PID_OUTPUT_LIMIT));

            setMotorPowers(fl, fr, bl, br);

            telemetry.addData("Robot X", "%.1f (Tgt: %.1f)", robotX, target.x);
            telemetry.addData("Robot Y", "%.1f (Tgt: %.1f)", robotY, target.y);
            telemetry.addData("Robot Hdg", "%.1f deg (Tgt: %.1f deg)",
                    ODOMETRY_ANGLE_UNIT.toDegrees(robotHeading), ODOMETRY_ANGLE_UNIT.toDegrees(target.heading));
            telemetry.addData("Dist Err", "%.1f", Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)));
            telemetry.addData("Head Err", "%.1f deg", ODOMETRY_ANGLE_UNIT.toDegrees(errorHeading)); // show signed error
            telemetry.addData("Pows", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f", fl, fr, bl, br);
            telemetry.addData("Loop Time(ms)", "%.3f", timeoutTimer.milliseconds() - lastLoopmSec);
            lastLoopmSec = timeoutTimer.milliseconds();
            telemetry.update();
            idle(); // Yield
        }

        // After loop: either at waypoint, timed out, or OpMode stopped
        if (target.stopAtWaypoint || timeoutTimer.seconds() >= NAVIGATION_TIMEOUT_SECONDS || !opModeIsActive()) {
            stopRobot();
            if (opModeIsActive() && target.stopAtWaypoint && timeoutTimer.seconds() < NAVIGATION_TIMEOUT_SECONDS) {
                sleep(500); // Pause if we stopped at waypoint as intended
            }
        }
        // If !target.stopAtWaypoint and we are at waypoint, we just continue (don't stop or sleep)
    }

    private void initializeDriveMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_CONFIG_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_CONFIG_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_CONFIG_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_CONFIG_NAME);

        transfer = hardwareMap.get(CRServo.class, TRANSFER_CONFIG_NAME);
        middle = hardwareMap.get(CRServo.class, MIDDLE_CONFIG_NAME);
        intake = hardwareMap.get(DcMotor.class, INTAKE_CONFIG_NAME);


        // YOUR MOTOR DIRECTIONS (ensure these are correct for your robot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        middle.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Drive Motors Initialized");
    }

    private void initializeGoBildaPinpoint() {
        try {
            pinpointOdometry = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_CONFIG_NAME);

            pinpointOdometry.setOffsets(X_OFFSET_TO_Y_DEADWHEEL_MM, Y_OFFSET_TO_X_DEADWHEEL_MM, ODOMETRY_DISTANCE_UNIT);  //see gobilda pinpoint manual

            pinpointOdometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            pinpointOdometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

            telemetry.addData("1st IMU Cal Status", "Calibrating IMU...");
            telemetry.update();
            pinpointOdometry.recalibrateIMU(); // This is a blocking call
            while (opModeIsActive() && pinpointOdometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
                telemetry.addData("1st IMU Cal Status", "IMU Still Calibrating...");
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("1st IMU Cal Status", "IMU Calibrated. Resetting Position...");
            telemetry.update();

            pinpointOdometry.resetPosAndIMU(); // Resets to 0,0,0 field origin and calibrates again
            while (opModeIsActive() && pinpointOdometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
                telemetry.addData("2nd IMU Cal Status", "IMU Still Calibrating...");
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("2nd IMU Cal Status", "IMU Calibrated. Resetting Position...");

            Pose2D initialFieldPose = new Pose2D(ODOMETRY_DISTANCE_UNIT, START_X_VALUE, START_Y_VALUE, ODOMETRY_ANGLE_UNIT, START_HEADING_VALUE);
            pinpointOdometry.setPosition(initialFieldPose); // Set the robot's starting position on the field
            sleep(100); // Allow position to "take"

            updateOdometryVariablesFromPinpoint(); // Update our local vars immediately with the set pose

            telemetry.addData("Status", "GoBildaPinpoint Initialized. Start Pose Set.");
            telemetry.addData("Initial Pose", "X:%.1f, Y:%.1f, H:%.1f deg", robotX, robotY, ODOMETRY_ANGLE_UNIT.toDegrees(robotHeading));


        } catch (Exception e) {
            telemetry.addData("Error", "Could not initialize GoBildaPinpointDriver: " + e.getMessage());
            pinpointOdometry = null; // Mark as unavailable
        }
    }

    private void initializePIDs() {
        xPidController = new PIDController(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
        yPidController = new PIDController(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
        headingPidController = new PIDController(P_HEADING_COEFF, I_HEADING_COEFF, D_HEADING_COEFF);

        // You might want to set output limits for PIDs if your PIDController class supports it
        // e.g., xPidController.setOutputLimits(-DRIVE_PID_OUTPUT_LIMIT, DRIVE_PID_OUTPUT_LIMIT);
        // However, the power scaling in navigateToTargetWaypoint handles overall magnitude.
    }

    private boolean updateOdometryVariablesFromPinpoint() {
        if (pinpointOdometry == null) return (false);

        pinpointOdometry.update();
        Pose2D pos = pinpointOdometry.getPosition();
        robotX = pos.getX(ODOMETRY_DISTANCE_UNIT);
        robotY = pos.getY(ODOMETRY_DISTANCE_UNIT);
        robotHeading = pos.getHeading(ODOMETRY_ANGLE_UNIT);  //getPosition  returns heading always normalized
        return (true);
    }

    private void displayTelemetry() {
        telemetry.addData("Actual Power ", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f",
                frontLeft.getPower(),
                frontRight.getPower(),
                backLeft.getPower(),
                backRight.getPower());
        telemetry.addData("Robot X", "%.2f %s", robotX, ODOMETRY_DISTANCE_UNIT.toString());
        telemetry.addData("Robot Y", "%.2f %s", robotY, ODOMETRY_DISTANCE_UNIT.toString());
        telemetry.addData("Robot Heading (Norm)", "%.2f deg", ODOMETRY_ANGLE_UNIT.toDegrees(robotHeading));


        if (pinpointOdometry != null) {
            telemetry.addData("Pinpoint Status", pinpointOdometry.getDeviceStatus());
            telemetry.addData("Pinpoint LoopTime (us)", pinpointOdometry.getLoopTime());
        }

        //telemetry.addData("Current Waypoint", (currentWaypointIndex == -1) ? "None" : (currentWaypointIndex + 1) + "/" + waypoints.length);
        //if (currentWaypointIndex != -1) {
        //    telemetry.addData("Target X", "%.2f %s", waypoints[currentWaypointIndex].x, ODOMETRY_DISTANCE_UNIT.toString());
        //    telemetry.addData("Target Y", "%.2f %s", waypoints[currentWaypointIndex].y, ODOMETRY_DISTANCE_UNIT.toString());
        //    telemetry.addData("Target Heading", "%.2f deg", ODOMETRY_ANGLE_UNIT.toDegrees(waypoints[currentWaypointIndex].heading));
        //}
        telemetry.update();
    }

    private void handleManualDrive() {
        // YOUR ORIGINAL MANUAL DRIVE LOGIC
        double xInput = -gamepad1.left_stick_y; // Your 'xInput' was for forward/backward component from joystick
        double yInput = gamepad1.left_stick_x;  // Your 'yInput' was for strafe component from joystick
        double rxInput = gamepad1.right_stick_x;// Rotational input from joystick

        double currentBotHeading = this.robotHeading; // This is normalized RADIANS

        // YOUR ORIGINAL Field Centric Transformation.
        // powX_transformed and powY_transformed are the robot-centric commands
        // derived from field-centric joystick inputs after rotation by bot heading.
        double powX_transformed = xInput * Math.cos(currentBotHeading) + yInput * Math.sin(-currentBotHeading);
        double powY_transformed = xInput * Math.sin(currentBotHeading) + yInput * Math.cos(-currentBotHeading);

        // YOUR ORIGINAL Mecanum mixing logic from manual drive.
        // It uses powX_transformed, powY_transformed, and rxInput.
        double denominator = Math.max(Math.abs(powY_transformed) + Math.abs(powX_transformed) + Math.abs(rxInput), 1.0);
        double frontLeftPower = (powX_transformed + powY_transformed + rxInput) / denominator;
        double backLeftPower = (powX_transformed - powY_transformed + rxInput) / denominator;
        double frontRightPower = (powX_transformed - powY_transformed - rxInput) / denominator;
        double backRightPower = (powX_transformed + powY_transformed - rxInput) / denominator;

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopRobot() {
        setMotorPowers(0, 0, 0, 0);
    }

    private boolean isAtWaypoint(Waypoint target) {
        double distanceError = Math.sqrt(Math.pow(target.x - robotX, 2) + Math.pow(target.y - robotY, 2));
        double headingError = Math.abs(normalizeAngle(target.heading - robotHeading));

        /****The following code w=x?y:z is shorthand for...
         if (x == true) { // or simply: if (target.stopAtWaypoint)
         w = y;
         } else {
         w = z;
         }
         ****/
        double currentDistanceTolerance = target.stopAtWaypoint ? DISTANCE_TOLERANCE_STOPPED : DISTANCE_TOLERANCE_PASSING;
        double currentHeadingTolerance = target.stopAtWaypoint ? HEADING_TOLERANCE_STOPPED : HEADING_TOLERANCE_PASSING;

        boolean atTranslation = distanceError < currentDistanceTolerance;
        boolean atRotation = headingError < currentHeadingTolerance;

        return atTranslation && atRotation; // Must satisfy both for stopping waypoints
    }

    /**
     * Normalizes an angle to be within the range of -Pi to +Pi radians.
     *
     * @param angle The angle in RADIANS.
     * @return The normalized angle in RADIANS.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // --- State Management ---
    private enum RobotState {
        IDLE,
        MANUAL_DRIVE,
        AUTONOMOUS_SEQUENCE_RUNNING
    }

    // --- Waypoint Definition ---
    private static class Waypoint {
        double x; // Target X in ODOMETRY_DISTANCE_UNIT
        double y; // Target Y in ODOMETRY_DISTANCE_UNIT
        double heading; // Target heading in ODOMETRY_ANGLE_UNIT (RADIANS, normalized)
        boolean stopAtWaypoint;

        public Waypoint(double x, double y, double heading, boolean stopAtWaypoint) {
            this.x = x;
            this.y = y;
            this.heading = heading; // Ensure this is always in RADIANS
            this.stopAtWaypoint = stopAtWaypoint;
        }
    }

    // Dummy PIDController class for completeness. Replace with your actual implementation.
    // Ensure its constructor and calculate() method match how they're used.
    private static class PIDController {
        private final double Kp;
        private final double Ki;
        private final double Kd;
        private double integralSum = 0;
        private double lastError = 0;
        private final ElapsedTime timer;

        public PIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.timer = new ElapsedTime();
        }

        public double calculate(double target, double current) {
            double error = target - current;
            double dt = timer.seconds();
            timer.reset();

            integralSum += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            lastError = error;

            return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            if (timer != null) { // timer might not be initialized if constructor wasn't called (e.g. static context issues)
                timer.reset();
            }
        }
    }
}
