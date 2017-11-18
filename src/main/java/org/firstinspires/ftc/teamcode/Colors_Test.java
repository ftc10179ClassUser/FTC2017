package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static com.sun.tools.doclint.Entity.divide;
import static com.sun.tools.doclint.Entity.rang;

/**
 * Created by Amy's on 10/27/2017.
 */



@TeleOp(name="Color Test", group="2017")
public class Colors_Test extends LinearOpMode {

    public static final double JEWEL_DOWN_POSITION = 0;
    public static final double JEWEL_UP_POSITION = 0;

    Range range;
    ColorSensor color;
    DcMotor right;
    DcMotor left;
    DcMotor center;
    Servo jewel;
    DcMotor wheel;
    Servo rservo;
    Servo lservo;
    public double leftSpeed;
    public double rightSpeed;
    public int speedCounter = 1;
    public double multiply;

    public void runOpMode() {


        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        center = hardwareMap.get(DcMotor.class, "center");
        jewel = hardwareMap.get(Servo.class, "jewel");
        rservo = hardwareMap.get(Servo.class, "rservo");
        lservo = hardwareMap.get(Servo.class, "lservo");
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        color = hardwareMap.get(ColorSensor.class, "color");
        range = hardwareMap.get(Range.class, "range");

        rservo.setPosition(.5);
        lservo.setPosition(.5);


        jewel.setPosition(0.35);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Color argb", color.argb());
            telemetry.addData("Color alpha", color.alpha());
            telemetry.addData("Color red", color.red());
            telemetry.addData("Color green", color.green());
            telemetry.addData("Color blue", color.blue());
            telemetry.update();
        }
        //get off platform
        //find two jewels
        while (color.blue() < 3){
            jewel.setPosition(JEWEL_DOWN_POSITION);
        }
    }
}
