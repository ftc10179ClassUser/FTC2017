package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import static android.R.attr.left;
import static android.R.attr.right;
import static android.R.attr.x;
import static android.R.transition.move;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RESET_ENCODERS;
import static com.sun.tools.doclint.Entity.divide;
import static com.sun.tools.javac.main.Option.S;
import static java.util.concurrent.TimeUnit.MILLISECONDS;

/*
────────────────────────────────────────────────────────────────────────────┐
This is a library for autonomous programs by Sam Knight. Read Autonomous_help.txt for more info.  |
────────────────────────────────────────────────────────────────────────────┘
*\

/**
 * Created by Amy's on 10/27/2017.
 */
@Autonomous(name = "Forward Test", group = "auto")
public class Forward_Test extends Autonomous_Library {

    boolean stop;

    DcMotor right;
    DcMotor left;
    DcMotor center;
    Servo jewel;
    DcMotor wheel;
    Servo rservo;
    Servo lservo;
    public int number = 0;
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


        rservo.setPosition(.5);
        lservo.setPosition(.5);

        telemetry.update();

        telemetry.update();
        waitForStart();





    }
}