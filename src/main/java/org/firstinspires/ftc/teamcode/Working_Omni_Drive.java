package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import static android.R.attr.y;
import static com.sun.tools.doclint.Entity.divide;
import static com.sun.tools.javac.main.Option.J;

@TeleOp (name="Working Omni Drive", group="2017")
public class Working_Omni_Drive extends LinearOpMode {

    double abs(double value) {
        if (value<0) {
            return value*-1;
        }

        return value;
    }

    private DcMotor right;
    private DcMotor left;
    private DcMotor center;
    private Servo jewel;
    public double leftSpeed;
    public double rightSpeed;
    public int turboCounter = 0;
    public double divide;

    public void runOpMode() {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        center = hardwareMap.get(DcMotor.class, "center");
        jewel = hardwareMap.get(Servo.class, "jewel");

        waitForStart();
        while (opModeIsActive()) {

            center.setPower(gamepad1.right_trigger);
            center.setPower(-1 * gamepad1.left_trigger);

            if (gamepad1.right_stick_button) {
                if (turboCounter == 0) {
                    turboCounter = 1;
                }

                if (turboCounter == 1) {
                    turboCounter = 0;
                }
            }

            leftSpeed = 0;
            rightSpeed = 0;

            double x = gamepad1.left_stick_x;
            double y = gamepad1.right_stick_y;

            leftSpeed += y;
            rightSpeed += y;

            if (gamepad1.right_bumper) {
                jewel.setPosition(.5);
            }

            if (gamepad1.left_bumper) {
                jewel.setPosition(0);
            }

            if (turboCounter == 1) {
                divide = 1;
            } else {
                divide = 4;
            }

            if (x<-0.1) {
                leftSpeed -= abs(x);
            }

            if (x>0.1) {
                rightSpeed -= abs(x);
            }

            right.setPower(rightSpeed/divide);
            left.setPower(leftSpeed/divide);
        }
    }
}