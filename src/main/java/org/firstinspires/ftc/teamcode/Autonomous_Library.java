package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

import static android.R.attr.left;
import static android.R.attr.right;
import static android.R.attr.x;
import static android.R.transition.move;
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
//@TeleOp(name="Maximum_Ogredrive", group="2017")
public class Autonomous_Library extends LinearOpMode {

    public static final int RUN_FOR_TIME = 0;

    boolean stop;

    DcMotor right1;
    DcMotor left1;
    DcMotor center1;
    Servo jewel1;
    DcMotor wheel1;
    Servo rservo1;
    Servo lservo1;
    public double leftSpeed;
    public double rightSpeed;
    public int speedCounter = 1;
    public double multiply;

    public void runOpMode() {


        right1 = hardwareMap.get(DcMotor.class, "right");
        left1 = hardwareMap.get(DcMotor.class, "left");
        center1 = hardwareMap.get(DcMotor.class, "center");
        jewel1 = hardwareMap.get(Servo.class, "jewel");
        rservo1 = hardwareMap.get(Servo.class, "rservo");
        lservo1 = hardwareMap.get(Servo.class, "lservo");
        wheel1 = hardwareMap.get(DcMotor.class, "wheel");
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        rservo1.setPosition(.5);
        lservo1.setPosition(.5);


        jewel1.setPosition(0.35);
    }

    public void servo(double position, Servo servo){
        servo.setPosition(position);
    }

    public void moveForwardSecs(double speed, double time, DcMotor left, DcMotor right) {
        tankMoveSecs(speed,speed,time,left,right);
    }

    public void moveForwardDegrees(double speed, int degrees, DcMotor left, DcMotor right) {
        tankMoveDegrees(speed,speed,degrees,left,right);
    }

    public void tankMoveSecs(double leftSpeed, double rightSpeed, double time, DcMotor leftMotor, DcMotor rightMotor) {
        moveMotorSecs(time, leftSpeed, leftMotor);
        moveMotorSecs(time, rightSpeed, rightMotor);
    }

    public void tankMoveDegrees(double leftSpeed, double rightSpeed, int degrees, DcMotor leftMotor, DcMotor rightMotor) {
        moveMotorSecs(degrees, leftSpeed, leftMotor);
        moveMotorSecs(degrees, rightSpeed, rightMotor);
    }

    public void moveMotorSecs(double secs, double power, DcMotor motor) {
        motor.setPower(power);
        if (wait(secs*1000)) {
            motor.setPower(0);
        }

    }


    public boolean wait(double milliseconds) {
        double x = System.nanoTime() + 1E6 * milliseconds;
        while (true) {
            if (x != -1 && x > System.nanoTime()) {
                return true;
            } else {
                x = -1;
            }
            if (stop) {
                return true;
            }
        }
    }

    public void moveMotorDegrees(int degrees, double power, DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(degrees);
        motor.setPower(power);
    }

    public void stopMotors(DcMotor left, DcMotor right) {
        stop = true;
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop = false;
    }

    public void goStraight(double power, int distance, DcMotor left, DcMotor right) {
        right.setTargetPosition(left.getCurrentPosition() + distance);
        left.setTargetPosition(left.getCurrentPosition() + distance);

        right.setPower(power);
        left.setPower(power);

        // set to run_to_position mode
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait until all motors reach position
        while (right.isBusy() || left.isBusy()) {
            telemetry.addLine("Is busy");
            telemetry.update();
        }

    }

    public void turnLeft(double power, int distance, DcMotor left, DcMotor right) {
        right.setTargetPosition(left.getCurrentPosition() - distance);
        left.setTargetPosition(left.getCurrentPosition() + distance);

        right.setPower(power);
        left.setPower(power);

        // set to run_to_position mode
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait until all motors reach position
        while (right.isBusy() || left.isBusy()) {
            telemetry.addLine("Is busy");
            telemetry.update();
        }

    }

    public void turnRight(double power, int distance, DcMotor left, DcMotor right) {
        right.setTargetPosition(left.getCurrentPosition() + distance);
        left.setTargetPosition(left.getCurrentPosition() - distance);

        right.setPower(power);
        left.setPower(power);

        // set to run_to_position mode
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait until all motors reach position
        while (right.isBusy() || left.isBusy()) {
            telemetry.addLine("Is busy");
            telemetry.update();
        }

    }

    public void strafe(double power, int distance, DcMotor strafeMotor) {
        strafeMotor.setTargetPosition(strafeMotor.getCurrentPosition() + distance);

        strafeMotor.setPower(power);

        // set to run_to_position mode
        strafeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait until all motors reach position
        while (strafeMotor.isBusy()) {
            telemetry.addLine("Is busy");
            telemetry.update();
        }

    }
}