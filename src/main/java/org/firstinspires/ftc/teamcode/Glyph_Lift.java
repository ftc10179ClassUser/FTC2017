package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.left;
import static android.R.attr.queryActionMsg;
import static com.sun.tools.doclint.Entity.divide;

/**
 * Created by Amy's on 10/27/2017.
 */
@TeleOp(name="Lift", group="2017")
public class Glyph_Lift extends LinearOpMode {

    DcMotor wheel;
    Servo rservo;
    Servo lservo;


    public void runOpMode() {

        rservo = hardwareMap.get(Servo.class, "rservo");
        lservo = hardwareMap.get(Servo.class, "lservo");
        wheel = hardwareMap.get(DcMotor.class, "wheel");

        rservo.setPosition(.5);
        lservo.setPosition(.5);

        waitForStart();
        while (opModeIsActive()) {

            if(gamepad2.left_stick_y>0) {
                if (gamepad2.left_stick_y > 0.25) {
                    wheel.setPower(gamepad2.left_stick_y);
                }
                if (gamepad2.left_stick_y < 0.25) {
                    wheel.setPower((gamepad2.left_stick_y) / 4);
                }
            }

            if(gamepad2.left_stick_y<0) {
                if (gamepad2.left_stick_y > -0.25) {
                    wheel.setPower((gamepad2.left_stick_y) / 4);
                }
                if (gamepad2.left_stick_y < -0.25) {
                    wheel.setPower(gamepad2.left_stick_y);
                }
            }

            if(gamepad2.left_bumper) {
                rservo.setPosition(.75);
                lservo.setPosition(.25);
            }
            if(gamepad2.right_bumper) {
                rservo.setPosition(.25);
                lservo.setPosition(.75);
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
