package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 10179 Tech Turtles on 10/27/2017.
 */

@TeleOp(name="Tech_Turtles_TeleOp", group="2017")
public class Omni_Drive_TeleOp extends LinearOpMode {

    //Initializes the motor names for the drive train.
    DcMotor right;
    DcMotor left;
    DcMotor center;

    //Initializes the servo name for the arm the removes the jewel.
    Servo jewel;

    //Initializes the motor name for the wheel that lifts the glyph mechanism.
    DcMotor pulley;

    //Initializes the servo names for the two arms on the glyph lifting mechanism.
    Servo rservo;
    Servo lservo;

    /*Initializes the name of the color sensor that is used for deciding which jewel to remove (this function is only used in a autonomous, but telemetry is found in teleop for
    finding color sensor values.
     */
    ColorSensor colorsensor;

    //Initializes the variables used.
    public boolean close = true;
    public boolean second_time = true;
    public boolean second_time2 = true;
    public boolean second_time3 = true;
    public boolean second_time4 = true;
    public boolean second_time5 = true;
    public int speedCounter = -1;
    public double multiply = 0.5;
    public double open_amount = 0.5;
    public double initAmount = 0.25;
    public boolean down;

    //This function decides whether the position of the glyph grabbing mechanism is open or closed.
    public void updateGlyphGrab() {
        //If the boolean close is true, the servos for the left and right arms move to their closed position.
        if (close) {
            rservo.setPosition(1);
            lservo.setPosition(0);
            //Otherwise they go to the open position   .
        } else {
            rservo.setPosition(open_amount+0.25);
            lservo.setPosition(open_amount-0.25);
        }
    }

    //This function decides the position of the arm that removes the jewel.
    public void updateJewelArm() {
        //If the boolean down is true, the servo moves to the down position.
        if (down) {
            jewel.setPosition(0);
            //Otherwise it goes to the up position.
        } else {
            jewel.setPosition(0.45);
        }
    }

    public void runOpMode() {

        //Assigns the device type to motors, servos, and sensors used and decides the name in the configuration.
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        center = hardwareMap.get(DcMotor.class, "center");
        jewel = hardwareMap.get(Servo.class, "jewel");
        rservo = hardwareMap.get(Servo.class, "rservo");
        lservo = hardwareMap.get(Servo.class, "lservo");
        pulley = hardwareMap.get(DcMotor.class, "wheel");
        colorsensor = hardwareMap.get(ColorSensor.class, "color");

        //Reverses the direction of the left motor on the drive train and of the pulley motor
        left.setDirection(DcMotor.Direction.REVERSE);
        pulley.setDirection(DcMotor.Direction.REVERSE);

        //Sets the position of the jewel removing arm and of the glyph lifting mechanism arms when initialized so that the robot is within the size requirements
        jewel.setPosition(0);
        rservo.setPosition(.7);
        lservo.setPosition(.35);
        int number = 0;

        //When the start button is pushed, the fallowing code is run
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("rservo", rservo.getPosition());
            telemetry.addData("lservo", lservo.getPosition());
            telemetry.update();

            //telemetry on the phone allows us to see the power that the motors are receiving and tells us what the blue and red values of the color sensor are for autonomous
            telemetry.addData("ryaxis", right.getPower());
            telemetry.addData("lyaxis", left.getPower());
            telemetry.addData("loop", number);
            telemetry.addData("LTrigger", gamepad1.left_trigger);
            telemetry.addData("RTrigger", gamepad1.right_trigger);
            telemetry.addData("red", colorsensor.red());
            telemetry.addData("blue", colorsensor.blue());
            telemetry.addData("Changes:", "Gamepad1's left bumper resets speed to turbo.\n Now, the robot does tank drive turns in slow mode.");
            telemetry.update();

            /*This is a element of our code that allows us to toggle the speed from full speed to half speed based on where we are on the field and whether the action we a
             doing requires precision. It starts out a default of half speed and pressing the left bumper will result in full speed, while pressing the right bumper will result
             in half speed.
              */
            if (gamepad1.right_bumper||gamepad1.left_bumper) {
                if (number == 0) {
                    if (gamepad1.right_bumper) {
                        number += 1;
                        speedCounter *= -1;
                        if (speedCounter == 1) {
                            multiply = 1;
                        } else if (speedCounter == -1) {
                            multiply = .5;
                        }
                    } else {
                        multiply = 1;
                    }
                }
            } else {
                number = 0;
            }

            //This section states how the drive train is controlled by saying that the power of the motors is equal to joy-sticks on controller 1
            double rightPower;
            double leftPower;

            float x_power = gamepad1.left_stick_x;
            float y_power = gamepad1.right_stick_y;

            rightPower = Range.clip(y_power, -1.0, 1.0);
            leftPower = Range.clip(x_power, -1.0, 1.0);

            //This section sets the power of the left and rights motors based on the variable multiply, or whether the controller sets it at full or half speed
            if (speedCounter == 1) {
                if (leftPower > 0) {
                    left.setPower(Range.clip((rightPower - leftPower) * multiply, -1.0, 1.0));
                    right.setPower(Range.clip(rightPower * multiply, -1.0, 1.0));
                } else {
                    left.setPower(Range.clip(rightPower * multiply, -1.0, 1.0));
                    right.setPower(Range.clip((rightPower + leftPower) * multiply, -1.0, 1.0));
                }
            } else {
                if (leftPower > 0) {
                    left.setPower(Range.clip((rightPower - leftPower) * multiply, -1.0, 1.0));
                    right.setPower(Range.clip((rightPower + leftPower) * multiply, -1.0, 1.0));
                } else {
                    left.setPower(Range.clip((rightPower - leftPower) * multiply, -1.0, 1.0));
                    right.setPower(Range.clip((rightPower + leftPower) * multiply, -1.0, 1.0));
                }
            }

            //This section sets the power of the center wheel while also applying the multiply variable that toggles the speed
            if (gamepad1.right_trigger>0){
                center.setPower(gamepad1.right_trigger*multiply);
            }
            else if (gamepad1.left_trigger>0) {
                center.setPower(-gamepad1.left_trigger*multiply);
            }
            else {
                center.setPower(0);
            }

            //This drive train is controlled used omni wheels, that allow us to strafe from side to side for easier access of the glyphs and other field components

            //This section raises and lowers the glyph lifting mechanism based on the input from the joy-stick on controller 2
            /*If the joy-stick is pressed upwards, the pulley wheel turns counter-clockwise, raising the glyph lift. If it is pressed upwards, then it turns clockwise, lowering the
             glyph lift. The value 0.1 is used to account for the dead-zone on the controller
              */
            if(gamepad2.left_stick_y>0.1) {
                //This if statement changes the power of the pulley when based on how far up the joy-stick is pressed
                if (gamepad2.left_stick_y > 0.25) {
                    pulley.setPower(gamepad2.left_stick_y);
                }
                if (gamepad2.left_stick_y < 0.25) {
                    pulley.setPower((gamepad2.left_stick_y) / 4);
                }
            } else if(gamepad2.left_stick_y<-0.1) {
                //This if statement changes the power of the pulley when based on how far up the joy-stick is pressed
                if (gamepad2.left_stick_y > -0.25) {
                    pulley.setPower((gamepad2.left_stick_y) / 4);
                }
                if (gamepad2.left_stick_y < -0.25) {
                    pulley.setPower(gamepad2.left_stick_y);
                }

                //If there is no input, the pulley wheel is not given any power
            } else {
                pulley.setPower(0);
            }

            /*This section opens and closes the glyph lifting mechanisms arms when the x button is pressed on the controller. By using the boolean second_time5 we can assign the
             functions of opening and closing to the same button for easier handling
              */
            if(gamepad1.x) {

                if (!second_time5) {
                    down = !down;
                    updateJewelArm();

                    second_time5 = true;
                }
            } else {
                second_time5 = false;
            }

            if(gamepad2.x) {
                if (!second_time) {
                    close = !close;
                    updateGlyphGrab();

                    second_time = true;
                }
            } else {
                second_time = false;
            }

            //In case the open and close positions of the glyph lifting mechanism's arms become out of sync, this resets them to the default
            if(gamepad2.left_bumper) {
                if (!second_time4) {
                    open_amount = 0.085;
                    updateGlyphGrab();

                    second_time4 = true;
                }
            } else {
                second_time4 = false;
            }

            //Based on how much room is available, this section allows the driver to toggle how wide the glyph lifting mechanism's arms can open

            //If the right trigger on the second controller is pressed, the updateGlyphGrab function is run again, except this time the servo positions for the open position increases
            if(gamepad2.right_trigger > 0.2) {
                if (!second_time2) {
                    open_amount += 0.05;
                    updateGlyphGrab();

                    second_time2 = true;
                }
            } else {
                second_time2 = false;
            }
            //If the left trigger on the second controller is pressed, the updateGlyphGrab function is run again, except this time the servo positions for the open position decreases
            if(gamepad2.left_trigger > 0.2) {
                if (!second_time3) {
                    open_amount -= 0.05;
                    updateGlyphGrab();

                    second_time3 = true;
                }
            } else {
                second_time3 = false;
            }
        }
    }
}