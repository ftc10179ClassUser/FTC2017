package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import static com.sun.tools.doclint.Entity.divide;
import static com.sun.tools.doclint.Entity.mu;


/**
 * Created by Amy's on 10/27/2017.
 */

@TeleOp(name="Maximum_Ogredrive v1", group="2017")
public class Omni_Test extends LinearOpMode {

    DcMotor right;
    DcMotor left;
    DcMotor center;
    Servo jewel;
    DcMotor pulley;
    Servo rservo;
    Servo lservo;
    public double leftSpeed;
    public double rightSpeed;
    public int speedCounter = 1;
    public double multiply = 0.5;

    public void runOpMode() {


        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        center = hardwareMap.get(DcMotor.class, "center");
        jewel = hardwareMap.get(Servo.class, "jewel");
        rservo = hardwareMap.get(Servo.class, "rservo");
        lservo = hardwareMap.get(Servo.class, "lservo");
        pulley = hardwareMap.get(DcMotor.class, "wheel");

        /*
        rservo.setPosition(.5);
        lservo.setPosition(.5);
        */

        rservo.setPosition(.25);
        lservo.setPosition(.5);

        left.setDirection(DcMotor.Direction.REVERSE);
        pulley.setDirection(DcMotor.Direction.REVERSE);

        jewel.setPosition(0.35);
        int number = 0;
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("ryaxis", right.getPower());
            telemetry.addData("lyaxis", left.getPower());
            telemetry.addData("loop", number);
            telemetry.update();

            if (gamepad1.left_stick_button) {
                if (number == 0) {
                    number += 1;
                    speedCounter *= -1;
                    if (speedCounter == 1) {
                        multiply = 1;
                    } else if (speedCounter == -1) {
                        multiply = .5;
                    }
                }
            } else {
                number = 0;
            }

            double rightPower;
            double leftPower;

            float x_power = gamepad1.left_stick_x;
            float y_power = gamepad1.right_stick_y;

            /*
            rightPower = Range.clip(y_power + x_power, -1.0, 1.0);
            leftPower = Range.clip(-y_power + x_power, -1.0, 1.0);
            */

            rightPower = Range.clip(y_power, -1.0, 1.0);
            leftPower = Range.clip(x_power, -1.0, 1.0);

            if (leftPower>0) {
                left.setPower(Range.clip((rightPower-leftPower)*multiply, -1.0, 1.0));
                right.setPower(Range.clip(rightPower*multiply, -1.0, 1.0));
            } else {
                left.setPower(Range.clip(rightPower*multiply, -1.0, 1.0));
                right.setPower(Range.clip((rightPower+leftPower)*multiply, -1.0, 1.0));
            }

            /*
            right.setPower(rightPower * (multiply));
            left.setPower(leftPower * (multiply));
            */

            center.setPower(gamepad1.right_trigger);
            center.setPower(-gamepad1.left_trigger);

            if (gamepad1.right_bumper) {
                jewel.setPosition(0.35);
            }

            if (gamepad1.left_bumper) {
                jewel.setPosition(0);
            }


            if(gamepad2.left_stick_y>0.1) {
                if (gamepad2.left_stick_y > 0.25) {
                    pulley.setPower(gamepad2.left_stick_y);
                }
                if (gamepad2.left_stick_y < 0.25) {
                    pulley.setPower((gamepad2.left_stick_y) / 4);
                }
            } else if(gamepad2.left_stick_y<-0.1) {
                if (gamepad2.left_stick_y > -0.25) {
                    pulley.setPower((gamepad2.left_stick_y) / 4);
                }
                if (gamepad2.left_stick_y < -0.25) {
                    pulley.setPower(gamepad2.left_stick_y);
                }
            } else {
                pulley.setPower(0);
            }

            if(gamepad2.left_bumper) {
                rservo.setPosition(1);
                lservo.setPosition(0);
            }
            if(gamepad2.right_bumper) {
                rservo.setPosition(.85);
                lservo.setPosition(.15);
            }

            if(gamepad2.a){
                rservo.setPosition(0);
                lservo.setPosition(0);
            }
            if(gamepad2.b){
                rservo.setPosition(1);
                lservo.setPosition(1);
            }


        }
    }
}