package Epsilon.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Epsilon.Superclasses.Subsystem;

public class IMU implements Subsystem {

    public BNO055IMU imu;
    private double angle;
    private double lastIMUReading;
    private static double zero = 0;

    public void initialize(LinearOpMode opMode) {
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        //while IMU is not calibrated
        while (opMode.opModeIsActive() && !imu.isGyroCalibrated());
        lastIMUReading = imu.getAngularOrientation().firstAngle;
    }

    public static double normalize(double angle) {
        // returns the angle geometrically equivalent to the input angle but between -180 and 180
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        }
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public void setZero(double zeroAngle){
        zero = zeroAngle;
    }

    public double zero(){
        return zero;
    }

    public double angle() {
        return angle + zero;
    }

    public void update() {
        double newAngle = imu.getAngularOrientation().firstAngle;
        angle += normalize(newAngle - lastIMUReading);
        lastIMUReading = newAngle;
    }

}