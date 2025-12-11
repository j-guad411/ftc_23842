package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class pid_test_tutorial extends OpMode {

    public DcMotorEx rightShoot;


   public double highVelocity =1500;


  public  double lowVelocity = 900;

  double curTargetVelocity= highVelocity;


  double F =0;

  double P =0;

  double[] stepSizes={10.0, 1.0, 0.01, 0.001, 0.0001};

  int stepIndex=1;





    @Override
    public void init() {
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightshoot");

    }

    @Override
    public void loop() {

    }
}
