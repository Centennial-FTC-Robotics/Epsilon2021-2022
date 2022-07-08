package Epsilon;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Epsilon.Subsystems.Drivetrain;
import Epsilon.Subsystems.CarouselJank;
import Epsilon.Subsystems.Intake;
import Epsilon.Subsystems.NoCap;
import Epsilon.Subsystems.Odometry;
import Epsilon.Subsystems.OpenCV;
import Epsilon.Subsystems.Outtake;
import Epsilon.Subsystems.IMU;
import Epsilon.Superclasses.EpsilonRobot;
import Epsilon.Superclasses.Subsystem;

//"OurRobot" class creates instances of all the subsystem
//This class also contains most of the methods we'll use in auto

public class OurRobot {

    //Creates instances of all the subsystem

    public static Odometry Odometry = new Odometry();
    public static Drivetrain drivetrain = new Drivetrain();
    public static CarouselJank carousel = new CarouselJank();
    public static Intake intake = new Intake();
    public static IMU imu = new IMU();
    public static Outtake outtake = new Outtake();
    public static OpenCV OpenCV = new OpenCV();
    public static NoCap capper = new NoCap();

    private static final Subsystem[] Subsystems = {
            outtake,
            drivetrain,
            intake,
            imu,
            Odometry,
            OpenCV,
            carousel,
            capper,
    };    //Array for all the subsystems


    // "initialize" method runs the "initialize" method in all the subsystems
    // Essentially declares/initializes all the motors and stuff
    public static void initialize(LinearOpMode opMode) {
        opMode.telemetry.addLine("Hi");
        opMode.telemetry.update();
        for (int i = 0; i<Subsystems.length; i++){
            Subsystems[i].initialize(opMode);
        }
    }

    public static void wait(int milliseconds, LinearOpMode opMode){
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < milliseconds && opMode.opModeIsActive()){

        }
    }
    public static void CycleFreight(LinearOpMode opMode) {
        //drive into warehouse
        drivetrain.Move(0.5, 35, Drivetrain.MoveType.DRIVE, opMode);

        //suck balls and cubes
        intake.wheel.setPower(1);
        wait(2500, opMode);

        //drive out and turn towards ASH
        drivetrain.Move(0.5, -50, Drivetrain.MoveType.DRIVE, opMode);
        drivetrain.Move(0.5, -18, Drivetrain.MoveType.TURN, opMode);

        //drive to ASH and score
        drivetrain.Move(0.5, -16, Drivetrain.MoveType.DRIVE, opMode);
        outtake.scoreASH(Outtake.PosASH.TOP);

        //drive back to starting position
        OurRobot.drivetrain.Move(0.5, 14, Drivetrain.MoveType.DRIVE, opMode);
        OurRobot.drivetrain.Move(0.5, 18, Drivetrain.MoveType.TURN, opMode);
        OurRobot.drivetrain.Move(0.5, 5, Drivetrain.MoveType.STRAFE, opMode);

        /*
        drivetrain.Move(0.5, 7, Drivetrain.MoveType.DRIVE, opMode);
        drivetrain.Move(0.5, -18, Drivetrain.MoveType.TURN, opMode);
        drivetrain.Move(0.5,18, Drivetrain.MoveType.STRAFE,opMode);
        drivetrain.Move(0.5,50, Drivetrain.MoveType.DRIVE,opMode);
        intake.wheel.setPower(-1);
        wait(5000, opMode);
        drivetrain.Move(0.5,-50, Drivetrain.MoveType.DRIVE,opMode);
        drivetrain.Move(0.5,-10, Drivetrain.MoveType.STRAFE,opMode);
        drivetrain.Move(0.5, 18, Drivetrain.MoveType.TURN, opMode);
        drivetrain.Move(0.5,-7, Drivetrain.MoveType.DRIVE,opMode);
        outtake.scoreASH(Outtake.PosASH.TOP);
        */
    }
}