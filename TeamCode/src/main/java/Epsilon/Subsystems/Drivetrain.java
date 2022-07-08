package Epsilon.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Base64;

import Epsilon.OurRobot;
import Epsilon.Superclasses.Subsystem;

//Initializes all the motors/hardware for the drivetrain
public class Drivetrain implements Subsystem {

    Odometry odo;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //PID constants - will be tuned to different values
    private double kP = 0.00008;
    private double minorkP = 0.00025;
    private double kI = 0.1;
    private double kD = 0.1;
    private double thres = 300;

    public void initialize(LinearOpMode opMode) {
        //odo = OurRobot.Odometry;

        frontLeft = opMode.hardwareMap.dcMotor.get("frontLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("frontRight");
        backLeft = opMode.hardwareMap.dcMotor.get("backLeft");
        backRight = opMode.hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Run without encoders because we'll probably be using Odo encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public enum MoveType {
        DRIVE,
        STRAFE,
        TURN
    }

    public void TargetPos(double EncoderCounts, MoveType Type) {
        switch (Type) {
            case DRIVE:
                frontLeft.setTargetPosition((int) EncoderCounts);
                frontRight.setTargetPosition((int) EncoderCounts);
                backLeft.setTargetPosition((int) EncoderCounts);
                backRight.setTargetPosition((int) EncoderCounts);
                break;
            case STRAFE:
                frontLeft.setTargetPosition((int) -EncoderCounts);
                frontRight.setTargetPosition((int) EncoderCounts);
                backLeft.setTargetPosition((int) EncoderCounts);
                backRight.setTargetPosition((int) -EncoderCounts);
                break;
            case TURN:
                frontLeft.setTargetPosition((int) -EncoderCounts);
                frontRight.setTargetPosition((int) EncoderCounts);
                backLeft.setTargetPosition((int) -EncoderCounts);
                backRight.setTargetPosition((int) EncoderCounts);
                break;
        }
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double INtoEC(double inches) {
        //Inches to Encoder Counts Stuff
        //(8192 ticks / 1 rev) * (1 rev / 1.96pi )
        double COUNTS_PER_INCH = 8192/(Math.PI*1.96);
        double EncoderCounts = inches * COUNTS_PER_INCH;
        return EncoderCounts;
    }

    public void Move(double power, int inches, MoveType Type, LinearOpMode opMode) {
        double EncoderCounts = INtoEC(inches);
        TargetPos(EncoderCounts, Type);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() && opMode.opModeIsActive()) {

        }
    }

    /*******************
     * PID Stuff Woohoo
     ******************/

    public void resetEncoderPos() {
        //odo.encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //odo.encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //odo.encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //odo.encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //might not even be necessary
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Basic PID method for linear/lateral movement
    public void Move(double inchesX, double inchesY, LinearOpMode opMode) {

        OurRobot.Odometry.update();
        double currentPosX = INtoEC(OurRobot.Odometry.xPos);
        double currentPosY = INtoEC(OurRobot.Odometry.yPos);
        double targetX = INtoEC(inchesX) + currentPosX;
        double targetY = INtoEC(inchesY) + currentPosY;
        double lastErrorX = 0;
        double lastErrorY = 0;
        double integralSumX = 0;
        double integralSumY = 0;
        double powerX = 0, powerY = 0;
        ElapsedTime timer = new ElapsedTime();
        while (!(targetX - currentPosX < thres && targetX - currentPosX > -thres
                && targetY - currentPosY < thres && targetY - currentPosY > -thres)) {

            OurRobot.Odometry.update();

            currentPosX = INtoEC(OurRobot.Odometry.xPos);
            currentPosY = INtoEC(OurRobot.Odometry.yPos);

            //calculate the error
            double errorX = targetX - currentPosX;
            double errorY = targetY - currentPosY;

            //ROC of the error
            double derivativeX = (errorX - lastErrorX) / timer.seconds();
            double derivativeY = (errorY - lastErrorY) / timer.seconds();

            //sum of all error over time
            integralSumX = integralSumX + (errorX * timer.seconds());
            integralSumY = integralSumY + (errorY * timer.seconds());

            if (inchesX > inchesY) {
                powerX = (kP * errorX);// + (kI * integralSumX) + (kD * derivativeX);
                powerY = (kP * errorY);// + (kI * integralSumY) + (kD * derivativeY);
            } else if (inchesX < inchesY) {
                powerX = (kP * errorX);// + (kI * integralSumX) + (kD * derivativeX);
                powerY = (kP * errorY);// + (kI * integralSumY) + (kD * derivativeY);
            }

            if (powerX < 0.1 && powerX > 0) {
                powerX = 0.1 * (errorX / Math.abs(errorX)); // determine pos or neg
            }
            if (powerY < 0.1 && powerY > 0) {
                powerY = 0.1 * (errorY / Math.abs(errorY));
            }
            //Power(power, Type);

            frontLeft.setPower(powerY - powerX);
            backLeft.setPower(powerY + powerX);
            frontRight.setPower(powerY + powerX);
            backRight.setPower(powerY - powerX);

            lastErrorX = errorX;
            lastErrorY = errorY;
            opMode.telemetry.addData("Target - Current (X)", targetX - currentPosX);
            opMode.telemetry.addData("Target - Current (Y)", targetY - currentPosY);
            opMode.telemetry.update();
            timer.reset();
        }
    }
}
