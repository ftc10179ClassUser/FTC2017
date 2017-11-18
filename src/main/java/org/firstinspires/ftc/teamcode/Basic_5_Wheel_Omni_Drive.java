package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name="OmniDrive", group="2017")
public class Basic_5_Wheel_Omni_Drive extends LinearOpMode {
    private DcMotor right;
    private DcMotor left;
    private DcMotor center;
    private Servo jewel;
    public double leftSpeed;
    public double rightSpeed;
    public int turboCounter = 1;

    /*public void leftMotorPower(double power) {
        leftSpeed += power / (turboCounter * 2);

    }

    public void rightMotorPower(double power) {
        rightSpeed += power / (turboCounter * 2);
    }*/

    public void runOpMode() {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        center = hardwareMap.get(DcMotor.class, "center");
        jewel = hardwareMap.get(Servo.class, "jewel");
        double threshold;
        threshold = 0.75;

        waitForStart();
        while (opModeIsActive()) {



            if(turboCounter==1) {
                right.setPower(gamepad1.right_stick_y);
                left.setPower(gamepad1.right_stick_y);
            }

            if(turboCounter==2) {
                right.setPower(gamepad1.right_stick_y/4);
                left.setPower(gamepad1.right_stick_y/4);
            }

            center.setPower(gamepad1.right_trigger);
            center.setPower(-1 * gamepad1.left_trigger);

            if (gamepad1.right_bumper) {
                jewel.setPosition(.5);
            }

            if (gamepad1.left_bumper) {
                jewel.setPosition(0);
            }

            if (gamepad1.left_stick_x < -0.05) {
                if (gamepad1.left_stick_x > (-1 * threshold)) {
                    right.setPower(-.5);
                }
                if (gamepad1.left_stick_x < (-1 * threshold)) {
                    right.setPower(-.5);
                    left.setPower(.5);
                }
            }

            if (gamepad1.left_stick_x > 0.05) {
                if (gamepad1.left_stick_x < (1 * threshold)) {
                    left.setPower(-.5);
                }
                if (gamepad1.left_stick_x > (1 * threshold)) {
                    left.setPower(-.5);
                    right.setPower(.5);
                }
            }

            if (gamepad1.left_stick_button) {
                if (turboCounter == 1) {
                    turboCounter = 2;
                }

                if (turboCounter == 2) {
                    turboCounter = 1;
                }
            }

            //right.setPower(rightSpeed);
            //left.setPower(leftSpeed);
        }
    }
}