package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

import static android.R.transition.move;
import static com.sun.tools.doclint.Entity.not;

//Created by 10179 Tech Turtles on 11/10/2017.
//Written by Sam Knight and Gabe Lerner-Sperow
//Comments by Gabe Lerner-Sperow

@Autonomous(name = "Blue Jewel Bottom", group = "auto")
public class autoJewelBlueBottom extends Autonomous_Library {

    //Initializes the motor names for the drive train.
    DcMotor right;
    DcMotor left;
    DcMotor center;
    double scannedDistance;
    double cryptoTurn;
    double cryptoTurn1;
    //Initializes the servo name for the arm the removes the jewel.
    Servo jewel;

    //Initializes the motor name for the wheel that lifts the glyph mechanism.
    DcMotor pulley;

    //Initializes the servo names for the two arms on the glyph lifting mechanism.
    Servo rservo;
    Servo lservo;

    //Initializes the name of the color sensor that is used for deciding which jewel to remove
    ColorSensor sensorColor2;
    ColorSensor sensorColor1;
    DistanceSensor sensorDistance2;
    DistanceSensor sensorDistance1;





    //vuValue is the value scanned by VuForia (L, C, or R)
    char vuValue;

    public void runOpMode() {



        // Wait until we're told to go


        // Start the logging of measured acceleration

        pulley = hardwareMap.get(DcMotor.class, "wheel");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "color2");
        sensorColor1 = hardwareMap.get(ColorSensor.class, "color1");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "color2");
        sensorDistance1 = hardwareMap.get(DistanceSensor.class, "color1");
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
        if (sensorColor2.red() < (sensorColor2.blue() + 15)) {
            telemetry.addData("youhere", "oops");
            telemetry.update();
            move(left, right, center, 0.3, 0.3, 0, 45, "lturn");
            jewel.setPosition(0);
            move(left, right, center, 0.3, 0.3, 0, 45, "rturn");

        }

        //If the amount of red seen by the sensor is greater than the amount of blue seen, then the robot will turn to the right
        else if (sensorColor2.blue() > (sensorColor2.red() + 15)) {
            telemetry.addData("youhere", "oops");
            telemetry.update();
            move(left, right, center, 0.3, 0.3, 0, 45, "rturn");
            jewel.setPosition(0);
            move(left, right, center, 0.3, 0.3, 0, 45, "lturn");
        }

        /*This section sets up VuForia so that it can scan the glyphs and give the phones telemetry on whether or not
        the VuMark can be seen.*/
        setUpVuforia();
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        RelicRecoveryVuMark vuMarkRB;
        vuMarkRB = RelicRecoveryVuMark.from(relicTemplate);
        //Please keep in mind that this, or any code related to VuForia is NOT owned or created by the 10179 Tech Turtles

        //While no VuMark is detected, the phone will keep scanning
        while (vuMarkRB == RelicRecoveryVuMark.UNKNOWN) {
            vuMarkRB = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMarkRB != RelicRecoveryVuMark.UNKNOWN) {

                //This assigns char values to the VuMark value detected for easier use later in the code
                switch (vuMarkRB) {
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

                telemetry.addData("VuMark", "%s visible", vuMarkRB);

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


        move(left, right, center, 0.3, 0.3, 0, 90, "rturn");
        jewel.setPosition(0);
        sleep(300);
        pulley.setPower(1);
        sleep(1000);
        exitBalancingStone();



        //It moves backwards to give it enough room to position for scoring a glyph


        /*Using the predetermined VuMark values, the the code determines what distance it must strafe to place the glyph into the
        correct column
         */
        switch (vuValue) {
            case 'L':
                //move in position to score into the left cryptobox
                move(left, right, center, 0.5, 0.5, 0, 5, "forward");
                break;

            case 'C':
                //move in position to score into the center cryptobox
                move(left, right, center, 0.5, 0.5, 0, 15, "forward");
                break;

            case 'R':
                //move in position to score into the right cryptobox
                move(left, right, center, 0.5, 0.5, 0, 25, "forward");
                break;

            default:
                //even if no value was found for the VuMark, the robot will still score the glyph into one of the columns
                move(left, right, center, 0.5, 0.5, 0, 8, "forward");
                break;
        }

        pulley.setPower(1);
        sleep(200);
        pulley.setPower(0);
        //turns around so that the side holding the glyph is facing towards the cryptobox

        move(left, right, center, 0.4, 0.4, 0, 80, "lturn");
        //go forward and score

        jewel.setPosition(0.45);
        sleep(500);
        /*while(abs(0.45-jewel.getPosition()) < 0.1) {
            move(left, right, center, 0.5, 0.5, 0, 0.5, "backward");
        }*/

        cryptoCorrection(scannedDistance*0.394);
        sleep(500);
        //turns around so that the side holding the glyph is facing towards the cryptobox
        move(left, right, center, 0.4, 0.4, 0, 190, "lturn");
        sleep(500);
        //go forward and score
        move(left, right, center, 0.5, 0.5, 0.5, 10., "backward");
        sleep(500);

        //open up
        rservo.setPosition(.6);
        lservo.setPosition(.45);
        sleep(500);

        //back up and park in the safe zone
        move(left, right, center, 1, 1, 0, 4, "forward");
        move(left, right, center, 1, 1, 0, 5, "backward");
        move(left, right, center, 1, 1, 0, 4, "forward");

    }
    public void cryptoCorrection(double cryptoDistance) {
        telemetry.addData("measurement", (sensorDistance2.getDistance(DistanceUnit.CM)));

        double moveAmount = (2.4 - cryptoDistance)*6;
        telemetry.addData("distance", moveAmount);
        telemetry.update();
        pulley.setPower(1);
        sleep(500);
        while (abs(2.4-(sensorDistance2.getDistance(DistanceUnit.CM)/2.54)) > 0.2) {
            if (cryptoDistance < 2.4) {
                center.setPower(0.5);
            }
            if (cryptoDistance > 2.4) {
                center.setPower(-0.5);
            }
            cryptoDistance = (sensorDistance2.getDistance(DistanceUnit.CM)/2.54);
        }
        center.setPower(0);
        jewel.setPosition(0);
    }
    public void turnCorrection() {
        telemetry.addData("turn correcting", "turning");
        telemetry.update();
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        while(abs(sensorDistance.getDistance(DistanceUnit.CM) - sensorDistance1.getDistance(DistanceUnit.CM)) < 0.2) {
            if(cryptoTurn > cryptoTurn1) {
                right.setPower(-0.3);
                left.setPower(0.3);
            }
            if(cryptoTurn < cryptoTurn1) {
                right.setPower(0.3);
                left.setPower(-0.3);
            }
            cryptoTurn = sensorDistance.getDistance(DistanceUnit.CM);
            cryptoTurn1 = sensorDistance1.getDistance(DistanceUnit.CM);
        }
        left.setPower(0);
        right.setPower(0);
    }

    public void exitBalancingStone() {
        while (sensorColor1.blue() > 20) {
            right.setDirection(DcMotorSimple.Direction.FORWARD);
            left.setDirection(DcMotorSimple.Direction.FORWARD);
            right.setPower(0.3);
            left.setPower(-0.3);
        }
        right.setPower(0);
        left.setPower(0);
        /*
        while (-90-logGyro < -2 || -90-logGyro > 2) {
            if(-90-logGyro < -2) {
                right.setPower(-0.5);
                left.setPower(-0.5);
                }
            if(-90-logGyro > 2) {
                right.setPower(0.5);
                left.setPower(0.5);
            }
        }*/
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}


//--------------------------------------\\
//      Autonomous Total Points: 85     \\
//--------------------------------------\\