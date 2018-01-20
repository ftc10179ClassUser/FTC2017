package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static android.R.transition.move;
import static com.sun.tools.doclint.Entity.not;

//Created by 10179 Tech Turtles on 11/10/2017.
//Written by Sam Knight and Gabe Lerner-Sperow
//Comments by Gabe Lerner-Sperow

@Autonomous(name = "Gyro Turn Test", group = "auto")
public class GyroTurnTest extends Autonomous_Library {

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

    //Initializes the name of the color sensor that is used for deciding which jewel to remove
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    //vuValue is the value scanned by VuForia (L, C, or R)
    char vuValue;
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    public double multiply;
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Sets the position of the jewel removing arm and of the glyph lifting mechanism arms when initialized so that the robot is within the size requirements

        move(left, right, center, 1, 1, 0, 180, "rturn");
    }
}

//--------------------------------------\\
//      Autonomous Total Points: 85     \\
//--------------------------------------\\