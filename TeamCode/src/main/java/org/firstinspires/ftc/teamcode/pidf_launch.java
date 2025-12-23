package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class pidf_launch extends OpMode {
    public DcMotorEx leftshoot;
    public DcMotorEx rightshoot;
    Limelight3A limelight;
    private DcMotor left_front_drive;
    private DcMotor right_front_drive;
    private DcMotor left_rear_drive;
    private DcMotor right_rear_drive;;
    private CRServo light;
    private CRServo middle;
    private DcMotor intake;
    private CRServo transfer;

    public double highVelocity = 1220;

    public double lowvelocity = 900;

    double curTargetVelocity = highVelocity;


    double F = 10;

    double P = 220;

    double[] stepsizes = {10, 1, 0.1, 0.01, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");

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

        leftshoot = hardwareMap.get(DcMotorEx.class, "left shoot");
        rightshoot = hardwareMap.get(DcMotorEx.class, "rightshoot");
        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftshoot.setDirection(DcMotor.Direction.REVERSE);
        rightshoot.setDirection(DcMotor.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init complete");
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y; // Forward/Backward
        double strafe = gamepad1.left_stick_x; // Strafe Left/Right
        double turn = gamepad1.right_stick_x; // Turn Left/Right


        telemetry.update();

        if (gamepad1.left_bumper) {
            intake.setPower(1);
            middle.setPower(1);
        }

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


        if (gamepad1.yWasPressed())
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowvelocity;
                light.setPower(1);
            } else {
                curTargetVelocity = highVelocity;
                light.setPower(.000002);
            }

        if (gamepad1.dpadUpWasPressed()) {
            P += 10;
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= 10;
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += 10;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= 10;
        }
        telemetry.addData("p", P);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftshoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        double leftcurVelocity = rightshoot.getVelocity();
        double rightcurVelocity = leftshoot.getVelocity();

        rightshoot.setVelocity(curTargetVelocity);
        leftshoot.setVelocity(curTargetVelocity);

        double lefterror = curTargetVelocity - rightcurVelocity;
        double righterror = curTargetVelocity - leftcurVelocity;

        telemetry.addData("right velo", rightshoot.getVelocity());
        telemetry.addData("target", curTargetVelocity);
        telemetry.addData("f", F);
    }
    }





