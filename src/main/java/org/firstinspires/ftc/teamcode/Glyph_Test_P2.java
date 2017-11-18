package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.ModernRoboticsI2cColorSensor2;

/**
 * Created by Amy's on 11/10/2017.
 */

@Autonomous(name = "autoClose", group = "auto")
public class Glyph_Test_P2 extends Autonomous_Library {

    Servo rservo;
    Servo lservo;

    public void runOpMode() {

        rservo = hardwareMap.get(Servo.class, "rservo");
        lservo = hardwareMap.get(Servo.class, "lservo");

        rservo.setPosition(1);
        lservo.setPosition(0);

    }
}