package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

import static android.R.transition.move;
import static com.sun.tools.doclint.Entity.not;

//Created by 10179 Tech Turtles on 11/10/2017.
//Written by Sam Knight and Gabe Lerner-Sperow
//Comments by Gabe Lerner-Sperow

@Autonomous(name = "Crypto Align Test", group = "auto")
public class CAT extends Autonomous_Library {

    //Initializes the motor names for the drive train.
    DcMotor right;
    DcMotor left;
    DcMotor center;
    double scannedDistance;
    //Initializes the servo name for the arm the removes the jewel.
    Servo jewel;

    //Initializes the motor name for the wheel that lifts the glyph mechanism.
    DcMotor pulley;

    //Initializes the servo names for the two arms on the glyph lifting mechanism.
    Servo rservo;
    Servo lservo;

    //Initializes the name of the color sensor that is used for deciding which jewel to remove
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    //vuValue is the value scanned by VuForia (L, C, or R)
    char vuValue;

    public void runOpMode() {


        pulley = hardwareMap.get(DcMotor.class, "wheel");
        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        center = hardwareMap.get(DcMotor.class, "center");
        jewel = hardwareMap.get(Servo.class, "jewel");
        rservo = hardwareMap.get(Servo.class, "rservo");
        lservo = hardwareMap.get(Servo.class, "lservo");


        //Sets the position of the jewel removing arm and of the glyph lifting mechanism arms when initialized so that the robot is within the size requirements
        rservo.setPosition(.7);
        lservo.setPosition(.35);
        jewel.setPosition(0);

        //This reverses the direction of the right motor
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        //The rest of the code will not run until the autonomous period begins and the start button is pressed on the phone
        waitForStart();

        rservo.setPosition(1);
        lservo.setPosition(0.025);
        sleep(500);

        pulley.setPower(1);
        sleep(1000);
        pulley.setPower(0);

        jewel.setPosition(0.45);
        sleep(5000);
        cryptoCorrection(scannedDistance*0.394);
        sleep(500);
        //turns around so that the side holding the glyph is facing towards the cryptobox
        move(left, right, center, 0.4, 0.4, 0, 177.5, "rturn");
        sleep(500);
        //go forward and score
        move(left, right, center, 0.5, 0.5, 0, 15, "backward");
        sleep(500);

        //open up
        rservo.setPosition(.6);
        lservo.setPosition(.45);
        sleep(500);

        //back up and park in the safe zone
        move(left, right, center, 1, 1, 0, 4, "forward");
    }
    public void cryptoCorrection(double cryptoDistance) {
        telemetry.addData("measurement", (sensorDistance.getDistance(DistanceUnit.CM)));

        double moveAmount = (2.4 - cryptoDistance)*6;
        telemetry.addData("distance", moveAmount);
        telemetry.update();
        pulley.setPower(1);
        sleep(500);
        while (abs(2.4-(sensorDistance.getDistance(DistanceUnit.CM)/2.54)) > 0.2) {
            if (cryptoDistance > 2.4) {
                move(left, right, center, 0, 0, 0.75, 0.1, "lstrafe");
            }
            if (cryptoDistance < 2.4) {
                move(left, right, center, 0, 0, 0.75, 0.1, "rstrafe");
            }
            cryptoDistance = (sensorDistance.getDistance(DistanceUnit.CM)/2.54);
        }
        jewel.setPosition(0);
    }
}

//--------------------------------------\\
//      Total Autonomous Points: 85     \\
//--------------------------------------\\