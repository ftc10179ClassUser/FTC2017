/*
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous_Library;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.concurrent.locks.Lock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


 Created by Amy's on 11/10/2017.


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous_Library;

@Autonomous(name = "autoTopLeft", group = "auto")
public class autoTopLeft extends Autonomous_Library {

    DcMotor right;
    DcMotor left;
    DcMotor center;
    Servo jewel;
    DcMotor wheel;
    Servo rservo;
    Servo lservo;
    ColorSensor color;

    private static final int neverest_cpr = 1120;
    boolean keepGoing = true;
    int cnumber = colorx.colorNumber();

    public void runOpMode() {

        color = hardwareMap.get(ColorSensor.class, "color");
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        center = hardwareMap.get(DcMotor.class, "center");
        jewel = hardwareMap.get(Servo.class, "jewel");
        rservo = hardwareMap.get(Servo.class, "rservo");
        lservo = hardwareMap.get(Servo.class, "lservo");
        wheel = hardwareMap.get(DcMotor.class, "wheel");

        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rservo.setPosition(.5);
        lservo.setPosition(.5);

        waitForStart();

        while (keepGoing == true) {
            // put arm down
            jewel.setPosition(0);

            // go straight and stop
            goStraight(0.25, neverest_cpr / 4, left, right);
            stopMotors(left, right);

            //when red shift right
            if (false) {
                strafe(1, neverest_cpr, center );
            }
        }

    }



}
*/