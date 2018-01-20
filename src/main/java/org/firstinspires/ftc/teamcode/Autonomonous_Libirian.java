package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentificationTeam;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentificationTeam;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentificationTeam;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentificationTeam;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentificationTeam;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentificationTeam;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuMarkIdentification;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentificationTeam;

import java.text.DecimalFormat;
import java.util.Locale;


/**
 *Created by Amy's on 10/27/2017.
 */

//@TeleOp(name="Library", group="2017")
public class Autonomonous_Libirian extends ConceptVuMarkIdentificationTeam {

    public static final int movemode_TURN = 1;
    public static final int movemode_MOVE = 0;

    boolean stop;
    int encoder;

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


        right1 = hardwareMap.get(DcMotor.class, "right");
        left1 = hardwareMap.get(DcMotor.class, "left");
        center1 = hardwareMap.get(DcMotor.class, "center");
        jewel1 = hardwareMap.get(Servo.class, "jewel");
        rservo1 = hardwareMap.get(Servo.class, "rservo");
        lservo1 = hardwareMap.get(Servo.class, "lservo");
        wheel1 = hardwareMap.get(DcMotor.class, "wheel");
        rservo1.setPosition(.5);
        lservo1.setPosition(.5);


    }

    public void servo(double position, Servo servo) {
        servo.setPosition(position);
    }

    public void moveForwardSecs(double speed, double time, DcMotor left, DcMotor right) {
        tankMoveSecs(speed, speed, time, left, right);
    }

    public void moveForwardDegrees(double speed, int degrees, DcMotor left, DcMotor right) {
        tankMoveDegrees(speed, speed, degrees, left, right);
    }

    public void tankMoveSecs(double leftSpeed, double rightSpeed, double time, DcMotor leftMotor, DcMotor rightMotor) {
        moveMotorSecs(time, leftSpeed, leftMotor);
        moveMotorSecs(time, rightSpeed, rightMotor);
    }

    public void tankMoveDegrees(double leftSpeed, double rightSpeed, int degrees, DcMotor leftMotor, DcMotor rightMotor) {
        moveMotorDegrees(degrees, leftSpeed, leftMotor);
        moveMotorDegrees(degrees, rightSpeed, rightMotor);
    }

    public void moveMotorSecs(double secs, double power, DcMotor motor) {
        motor.setPower(power);
        if (wait(secs * 1000)) {
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

        while (motor.isBusy()) {
            telemetry.addLine("Moving to degrees...");
            telemetry.update();
        }
        telemetry.update();
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

    public void stopTest(DcMotor left, DcMotor right) {
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


    private double abs(double d) {
        if (d < 0) {
            return -d;
        } else {
            return d;
        }
    }

    private int round(double d) {
        double dAbs;

        if (d < 0) {
            dAbs = -d;
        } else {
            dAbs = d;
        }

        int i = (int) dAbs;
        double result = dAbs - (double) i;
        if (result < 0.5) {
            return d < 0 ? -i : i;
        } else {
            return d < 0 ? -(i + 1) : i + 1;
        }
    }

    public static DecimalFormat df2 = new DecimalFormat(".##");

    public void dround(long input) {
        df2.format(input);
    }

    //f = forward, tr = turn right, tl = turn left
    public void move(DcMotor left, DcMotor right, DcMotor center, double lpower, double rpower, double cpower, double distanceOrDegrees, String driveMode) {

        setUpGyro();
        String text = formatAngle(angles.angleUnit, angles.firstAngle);
        double errorAmount = Double.parseDouble(text);
        switch (driveMode) {
            case "forward":
                telemetry.addData("goesforward", 0);
                telemetry.update();
                right.setDirection(DcMotorSimple.Direction.FORWARD);
                left.setDirection(DcMotorSimple.Direction.FORWARD);
                right.setDirection(DcMotorSimple.Direction.REVERSE);
                right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int fencoder = (round(((distanceOrDegrees / 12.56) * (1680 / 2)) * .77));
                while (right.getCurrentPosition() < fencoder) {
                    showGyroTelemetry();
                    right.setPower(rpower);
                    left.setPower(lpower);
                    if (abs(0 - errorAmount) < 1) {
                        right.setPower(0);
                        left.setPower(0);
                        if (errorAmount < 0) {
                            while (abs(0 - errorAmount) < 1) {
                                right.setPower(0.3);
                            }
                            right.setPower(0);
                        } else {
                        }
                        if (errorAmount > 0) {
                            while (abs(0 - errorAmount) < 1) {
                                left.setPower(0.3);
                            }
                            left.setPower(0);
                        } else {
                        }
                    } else {
                    }
                }
                right.setPower(0);
                left.setPower(0);
                break;

            case "backward":
                telemetry.addData("goesbackward", 0);
                telemetry.update();
                right.setDirection(DcMotorSimple.Direction.FORWARD);
                left.setDirection(DcMotorSimple.Direction.FORWARD);
                left.setDirection(DcMotorSimple.Direction.REVERSE);
                right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int bencoder = (round(((distanceOrDegrees / 12.56) * (1680 / 2)) * .77));
                while (right.getCurrentPosition() < bencoder) {
                    showGyroTelemetry();
                    right.setPower(rpower);
                    left.setPower(lpower);
                    if (abs(0 - errorAmount) < 1) {
                        right.setPower(0);
                        left.setPower(0);
                        if (errorAmount < 0) {
                            while (abs(0 - errorAmount) < 1) {
                                left.setPower(0.3);
                            }
                            left.setPower(0);
                        } else {
                        }

                        if (errorAmount > 0) {
                            while (abs(0 - errorAmount) < 1) {
                                right.setPower(0.3);
                            }
                            right.setPower(0);
                        } else {
                        }
                    } else {
                    }
                }
                right.setPower(0);
                left.setPower(0);
                break;

            case "rturn":
                telemetry.addData("turnsright", 0);
                telemetry.update();
                right.setDirection(DcMotorSimple.Direction.FORWARD);
                left.setDirection(DcMotorSimple.Direction.FORWARD);
                right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int trencoder = (round(distanceOrDegrees * 8.7));
                while (right.getCurrentPosition() < trencoder) {
                    showGyroTelemetry();
                    right.setPower(rpower);
                    left.setPower(lpower);
                }
                showGyroTelemetry();
                right.setPower(0);
                left.setPower(0);
                showGyroTelemetry();
                if (errorAmount != distanceOrDegrees) {
                    if (errorAmount < distanceOrDegrees) {
                        while (errorAmount != distanceOrDegrees) {
                            right.setPower(rpower);
                            left.setPower(lpower);
                        }
                        right.setPower(0);
                        left.setPower(0);
                    } else {
                    }
                    if (errorAmount > distanceOrDegrees) {
                        while (errorAmount != distanceOrDegrees) {
                            right.setPower(-rpower);
                            left.setPower(-lpower);
                        }
                        right.setPower(0);
                        left.setPower(0);
                    }
                } else {
                }
                break;

            case "lturn":
                telemetry.addData("turnsleft", 0);
                telemetry.update();
                right.setDirection(DcMotorSimple.Direction.FORWARD);
                left.setDirection(DcMotorSimple.Direction.FORWARD);
                right.setDirection(DcMotorSimple.Direction.REVERSE);
                left.setDirection(DcMotorSimple.Direction.REVERSE);
                right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int tlencoder = (round(distanceOrDegrees * 8.7));
                while (right.getCurrentPosition() < tlencoder) {
                    showGyroTelemetry();
                    right.setPower(rpower);
                    left.setPower(lpower);
                    showGyroTelemetry();
                    if (errorAmount != distanceOrDegrees) {
                        if (errorAmount < distanceOrDegrees) {
                            while (errorAmount != distanceOrDegrees) {
                                right.setPower(rpower);
                                left.setPower(lpower);
                            }
                            right.setPower(0);
                            left.setPower(0);
                        } else {
                        }
                        if (errorAmount > distanceOrDegrees) {
                            while (errorAmount != distanceOrDegrees) {
                                right.setPower(-rpower);
                                left.setPower(-lpower);
                            }
                            right.setPower(0);
                            left.setPower(0);
                        }
                    } else {
                    }
                }

                right.setPower(0);
                left.setPower(0);
                break;
            case "lstrafe":
                telemetry.addData("strafesleft", 0);
                telemetry.update();
                center.setDirection(DcMotorSimple.Direction.FORWARD);
                center.setDirection(DcMotorSimple.Direction.REVERSE);
                center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                center.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int lsencoder = (round((((distanceOrDegrees / 12.56) * (288)) * .77) * 1.25 * 1.066));
                while (center.getCurrentPosition() < lsencoder) {
                    if (abs(center.getCurrentPosition() - lsencoder) < (50 * abs(cpower))) {
                        center.setPower(0.5);
                    } else {
                        center.setPower(cpower);
                    }
                }
                break;
            case "rstrafe":
                telemetry.addData("strafesright", 0);
                telemetry.update();
                center.setDirection(DcMotorSimple.Direction.FORWARD);
                center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                center.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int rsencoder = (round((((distanceOrDegrees / 12.56) * (288)) * .77) * 1.25 * 1.066));
                while (center.getCurrentPosition() < rsencoder) {
                    if (abs(center.getCurrentPosition() - rsencoder) < (50 * abs(cpower))) {
                        center.setPower(0.5);
                    } else {
                        center.setPower(cpower);
                    }
                    telemetry.addData("current position", center.getCurrentPosition());
                    telemetry.addData("threshhold", (200 * abs(cpower)));
                    telemetry.addData("going fast", !(abs(center.getCurrentPosition() - rsencoder) < (100 * abs(cpower))));
                    telemetry.update();
                }
                break;


            //moveMotorDegrees(round(encoder),power,left);
            //moveMotorDegrees(round(encoder),power,right);


        }
    }

    public void setUpVuforia() {
            /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */

        parameters.vuforiaLicenseKey = "AcuNtA7/////AAAAmY411Wf19002k5uGCVSx9Etw4/sJrBDz06eC+CTJqcia0KXAOu8HzPbmcDU9PdSW0qC/4qWEbGV8iuIdQhQUqR54xsoiNaXxWOgF02gkCxy7vHG9wvBtAIbieFLoSbkpU4KZry3BqCnemu9Z3FmnD2uOyhM7bXtPGvCcgn78cyIKDvZXfXftGCqR7VMl04yE2LSJpwWaw3AzOOovUKULIrl5oA2OX8BeF58ktVLTwXux8KnMkpM83H/XO1xGoTqTZ2Oo18BvTesJBGNcojtI9E0NtYA1dHgNUaFA/zPFS6rh3PrwK3PojSqQZ5e/Yd+698uQSvoSHBwx4Yiq6EKzFAXgobjX/t8Gwf9ttFlodt7Q";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    public void setUpGyro() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void showGyroTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}






