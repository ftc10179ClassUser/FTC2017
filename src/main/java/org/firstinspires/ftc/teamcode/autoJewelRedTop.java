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

import static android.R.transition.move;
import static com.sun.tools.doclint.Entity.not;

//Created by 10179 Tech Turtles on 11/10/2017.
//Written by Sam Knight and Gabe Lerner-Sperow
//Comments by Gabe Lerner-Sperow

@Autonomous(name = "Red Jewel Top", group = "auto")
public class autoJewelRedTop extends Autonomous_Library {

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

        //Insures that the drive train does not move yet
        left.setPower(0);
        right.setPower(0);

        rservo.setPosition(1);
        lservo.setPosition(0.025);
        sleep(500);

        pulley.setPower(1);
        sleep(1000);
        pulley.setPower(0);

        //This lowers the jewel removing arm to the down position so that the color/distance sensor can scan the ball
        jewel.setPosition(0.45);
        sleep(1000);

        //If the amount of blue seen by the sensor is greater than the amount of red seen, then the robot will turn to the left
        if (sensorColor.red() < (sensorColor.blue() + 15)) {
            telemetry.addData("youhere", "oops");
            telemetry.update();
            move(left, right, center, 0.3, 0.3, 0, 45, "rturn");
            jewel.setPosition(0);
            move(left, right, center, 0.3, 0.3, 0, 45, "lturn");
        }

        //If the amount of red seen by the sensor is greater than the amount of blue seen, then the robot will turn to the right
        else if (sensorColor.blue() < (sensorColor.red() + 15)) {
            telemetry.addData("youhere", "oops");
            telemetry.update();
            move(left, right, center, 0.3, 0.3, 0, 45, "lturn");
            jewel.setPosition(0);
            move(left, right, center, 0.3, 0.3, 0, 45, "rturn");
        }

        /*This section sets up VuForia so that it can scan the glyphs and give the phones telemetry on whether or not
        the VuMark can be seen.*/
        setUpVuforia();
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        RelicRecoveryVuMark vuMarkBB;
        vuMarkBB = RelicRecoveryVuMark.from(relicTemplate);
        //Please keep in mind that this, or any code related to VuForia is NOT owned or created by the 10179 Tech Turtles

        //While no VuMark is detected, the phone will keep scanning
        while (vuMarkBB == RelicRecoveryVuMark.UNKNOWN) {
            vuMarkBB = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMarkBB != RelicRecoveryVuMark.UNKNOWN) {

                //This assigns char values to the VuMark value detected for easier use later in the code
                switch (vuMarkBB) {
                    case LEFT:
                        vuValue = 'L';
                        break;
                    case CENTER:
                        vuValue = 'C';
                        break;
                    case RIGHT:
                        vuValue = 'R';
                        break;
                }

                telemetry.addData("VuMark", "%s visible", vuMarkBB);

            } else {
                telemetry.addData("VuMark", "not visible");
            }

            //Updates the telemetry, telling the user the status of VuMark scanning
            telemetry.update();

        }

        //This is necessary to insure that the glyph is not dragging on the ground as the program tries to run
        //By giving the glyph lift a short burst of power, we can make sure that it doesn't fall to the ground
        pulley.setPower(1);
        sleep(1000);
        pulley.setPower(0);
        move(left, right, center, 0.3, 0.3, 0, 95, "lturn");
        sleep(300);
        move(left, right, center, 0.5, 0.5, 0, 21, "forward");
        sleep(500);

        //It moves backwards to give it enough room to position for scoring a glyph


        /*Using the predetermined VuMark values, the the code determines what distance it must strafe to place the glyph into the
        correct column
         */
        switch (vuValue) {
            case 'R':
                //move in position to score into the left cryptobox
                move(left, right, center, 0, 0, 0.7, 5, "rstrafe");
                break;

            case 'C':
                //move in position to score into the center cryptobox
                move(left, right, center, 0, 0, 0.7, 18, "rstrafe");
                break;

            case 'L':
                //move in position to score into the right cryptobox
                move(left, right, center, 0, 0, 0.7, 23, "rstrafe");
                break;

            default:
                //even if no value was found for the VuMark, the robot will still score the glyph into one of the columns
                move(left, right, center, 0.5, 0.5, 0, 8, "forward");
                break;
        }

        sleep(500);
        pulley.setPower(1);
        sleep(200);
        pulley.setPower(0);
        //turns around so that the side holding the glyph is facing towards the cryptobox


        jewel.setPosition(0.45);
        sleep(500);
        /*while(abs(0.45-jewel.getPosition()) < 0.1) {
            move(left, right, center, 0.5, 0.5, 0, 0.5, "backward");
        }*/

        cryptoCorrection(scannedDistance*0.394);
        sleep(500);
        //turns around so that the side holding the glyph is facing towards the cryptobox
        move(left, right, center, 0.4, 0.4, 0, 190, "r" +
                "turn");
        sleep(500);
        //go forward and score
        move(left, right, center, 0.5, 0.5, 0.5, 10, "backward");
        sleep(500);

        //open up
        rservo.setPosition(.6);
        lservo.setPosition(.45);
        sleep(500);

        //back up and park in the safe zone
        move(left, right, center, 1, 1, 0, 4, "forward");
        move(left, right, center, 1, 1, 0, 4, "backward");
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
            if (cryptoDistance < 3) {
                center.setPower(-0.5);
            }
            if (cryptoDistance > 3) {
                center.setPower(0.5);
            }
            cryptoDistance = (sensorDistance.getDistance(DistanceUnit.CM)/2.54);
        }
        center.setPower(0);
        jewel.setPosition(0);

    }
}

//--------------------------------------\\
//      Autonomous Total Points: 85     \\
//--------------------------------------\\