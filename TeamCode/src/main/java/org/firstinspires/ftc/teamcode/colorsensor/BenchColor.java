package org.firstinspires.ftc.teamcode.colorsensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BenchColor {
    NormalizedColorSensor colorSensor;

    public enum Dectectedcolor {
        PURPLE,
        GREEN,
        UNKNOWN,


    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "shooter_stop");
        colorSensor.setGain(4);//change to get good value
    }

    public Dectectedcolor getDetectedColor(Telemetry telemtry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;


        telemtry.addData("red", normRed);
        telemtry.addData("blue", normBlue);
        telemtry.addData("green", normGreen);
        //TODO ADD IF STATEMENTS FOR SPECIFIC COLORS ADDED
        /*




         */

        return Dectectedcolor.UNKNOWN;
    }
}