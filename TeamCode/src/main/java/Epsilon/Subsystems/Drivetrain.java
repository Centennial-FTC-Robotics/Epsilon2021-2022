package Epsilon.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Base64;

import Epsilon.Superclasses.Subsystem;

//Initializes all the motors/hardware for the drivetrain
public class Drivetrain implements Subsystem {

    Odometry odo = new Odometry();

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //PID constants - will be tuned to different values
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    public void initialize(LinearOpMode opMode) {

        frontLeft = opMode.hardwareMap.dcMotor.get("frontLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("frontRight");
        backLeft = opMode.hardwareMap.dcMotor.get("backLeft");
        backRight = opMode.hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void Power(double power, MoveType Type) {
        switch (Type) {
            case DRIVE:
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
                break;
            case STRAFE:
                frontLeft.setPower(-1 * power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(-1 * power);
                break;
            case TURN:
                frontLeft.setPower(-1 * power);
                frontRight.setPower(power);
                backLeft.setPower(-1 * power);
                backRight.setPower(power);
                break;
        }
    }

    public double INtoEC(double inches) {       //Hey Jacob I changed from int to double
        //Inches to Encoder Counts Stuff
        double EncoderCounts = inches;
        return EncoderCounts;
    }
/*
    public void Move(double power, int inches, MoveType Type) {
        double EncoderCounts = INtoEC(inches);
        //Filler for setting Encoder Counts (this is for default motor encoders, not odo)
        frontLeft.setTargetPosition((int) EncoderCounts);
        frontRight.setTargetPosition((int) EncoderCounts);
        backLeft.setTargetPosition((int) EncoderCounts);
        backRight.setTargetPosition((int) EncoderCounts);
        //POWAAAAA
        Power(power, Type);
    }
*/
    /*******************
     * PID Stuff Woohoo
     ******************/

    public void resetEncoderPos(){
        odo.encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //might not even be necessary
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Basic PID method for linear movement
    public void Move(double inches, MoveType Type){

        double target = INtoEC(inches);
        double currentPos = odo.encoderX.getCurrentPosition();
        double lastError = 0;
        double integralSum = 0;

        ElapsedTime timer = new ElapsedTime();
        while (target - currentPos > 0){

            currentPos = odo.encoderX.getCurrentPosition();
            //calculate the error
            double error = target - currentPos;

            //ROC of the error
            double derivative  = (error - lastError) / timer.seconds();

            //sum of all error over time
            integralSum = integralSum + (error*timer.seconds());

            double power = (kP*error) + (kI*integralSum) + (kD*derivative);

            Power(power, Type);
/*
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
*/
            lastError = error;
            timer.reset();
        }
    }

}
