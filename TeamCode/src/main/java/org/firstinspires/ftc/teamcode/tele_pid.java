package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "MainTeleOp (Blocks to Java)")
public class tele_pid extends LinearOpMode {

    Limelight3A limelight;
    private DcMotor left_front_drive;
    private DcMotor right_front_drive;
    private DcMotor left_rear_drive;
    private DcMotor right_rear_drive;
     public DcMotorEx leftshoot;

    private CRServo light;
    private CRServo middle;
    private DcMotor intake;
    private CRServo transfer;
    public DcMotorEx rightshoot;
    public double highVelocity = 1265;

    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;

    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex=1;
    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override

    public void runOpMode() {


        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        leftshoot = hardwareMap.get(DcMotorEx.class, "left shoot");
        rightshoot = hardwareMap.get(DcMotorEx.class, "rightshoot");
        light = hardwareMap.get(CRServo.class, "light");
        middle = hardwareMap.get(CRServo.class, "middle");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        right_rear_drive.setDirection(DcMotor.Direction.FORWARD);
        leftshoot.setDirection(DcMotor.Direction.REVERSE);
        rightshoot.setDirection(DcMotor.Direction.FORWARD);
        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        rightshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        rightshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("int complete");



        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.


            while (opModeIsActive()) {
                double drive = -gamepad1.left_stick_y; // Forward/Backward
                double strafe = gamepad1.left_stick_x; // Strafe Left/Right
                double turn = gamepad1.right_stick_x; // Turn Left/Right
                if (gamepad1.dpad_up) {
                    ((DcMotorEx) leftshoot).setVelocity(+2);
                    ((DcMotorEx) rightshoot).setVelocity(-2);
                }
                if (gamepad1.dpad_up) {
                    ((DcMotorEx) leftshoot).setVelocity(-2);
                    ((DcMotorEx) rightshoot).setVelocity(+2);
                }
                telemetry.addData("shooter right velo", ((DcMotorEx)rightshoot).getVelocity());
                telemetry.addData("shooter left velo", ((DcMotorEx)leftshoot).getVelocity());
                telemetry.update();
            if (gamepad1.yWasPressed()) {
                if(curTargetVelocity==highVelocity){
                    curTargetVelocity=lowVelocity;
                }else {curTargetVelocity=highVelocity;}

            }

            if (gamepad1.bWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

                if (gamepad1.left_bumper) {
                    intake.setPower(1);
                    middle.setPower(1);
                }
                if (gamepad1.right_bumper) {
                    intake.setPower(0);
                    middle.setPower(0);
                }

                if (gamepad1.dpadLeftWasPressed()){
                    F -= stepSizes[stepIndex];
                }

                if (gamepad1.dpadRightWasPressed()){
                    F+= stepSizes[stepIndex];
                }
                if (gamepad1.dpadUpWasPressed()){
                    P += stepSizes[stepIndex];
                }
                if (gamepad1.dpadDownWasPressed()){
                    P-= stepSizes[stepIndex];
                }
               // PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                rightshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
                rightshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

                //set velocity
                rightshoot.setVelocity(curTargetVelocity);
                leftshoot.setVelocity(curTargetVelocity);

                double curvelocityRight = rightshoot.getVelocity();
                double curvelocityLeft = leftshoot.getVelocity();

                double righterror = curTargetVelocity - curvelocityRight;
                double lefterror = curTargetVelocity - curvelocityLeft;

                telemetry.addData("target velocity", curTargetVelocity);
                telemetry.addData("curent left velocity", "%.2f",curvelocityLeft);
                telemetry.addData("curent right velocity", "%.2f",curvelocityRight);
                telemetry.addData("righterror", "%.2", righterror);
                telemetry.addData("lefterror", "%.2", lefterror);
                telemetry.addLine("-----------------------------");
                telemetry.addData("tuning p", "%.4f(d-pad u/d)",P);
                telemetry.addData("tuning f", "%.4f(d-pad l/r)",F);
                telemetry.addData("step size", "%.4f (b buton)", stepSizes[stepIndex]);


                //set new pid coefficients



                if (gamepad1.right_bumper) {
                    middle.setPower(0);
                    intake.setPower(0);

                }
                if (gamepad2.left_bumper) {
                    transfer.setPower(-1);
                    middle.setPower(1);
                    intake.setPower(1);
                }
                if (gamepad2.right_bumper) {
                    transfer.setPower(0);
                    middle.setPower(0);
                    intake.setPower(0);
                }
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx(); // How far left or right the target is (degrees)
                    double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    telemetry.addData("tx: ", tx);
                    telemetry.addData("ta: ", ta);

                    double kP = 0.02;
                    double kF = 0.05;
                    double kFThreshold = 0.5;

                    if (gamepad1.right_trigger > .1) {
                        turn = (tx - 2) * kP;
                        if ((tx - 2) > kFThreshold) {
                            turn += kF;
                        }
                        if ((tx - 2) < -kFThreshold) {
                            turn -= kF;
                        }
                    }
                    if (gamepad1.left_trigger > .1) {
                        turn = (tx - 0) * kP;
                        if ((tx - 0) > kFThreshold) {
                            turn += kF;
                        }
                        if ((tx - 0) < -kFThreshold) {
                            turn -= kF;
                        }
                    }

                } else {
                    telemetry.addData("Limelight", "No Targets");
                }
                // Calculate motor powers
                double frontLeftPower = drive + strafe + turn;
                double backLeftPower = drive - strafe + turn;
                double frontRightPower = drive - strafe - turn;
                double backRightPower = drive + strafe - turn;

                // Normalize powers to keep them within -1 to 1 range
                double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                maxPower = Math.max(maxPower, Math.abs(frontRightPower));
                maxPower = Math.max(maxPower, Math.abs(backRightPower));

                if (maxPower > 1.0) {
                    frontLeftPower /= maxPower;
                    backLeftPower /= maxPower;
                    frontRightPower /= maxPower;
                    backRightPower /= maxPower;
                }

                // Set motor powers
                left_front_drive.setPower(frontLeftPower);
                left_rear_drive.setPower(backLeftPower);
                right_front_drive.setPower(frontRightPower);
                right_rear_drive.setPower(backRightPower);


            }


        }
    }
}