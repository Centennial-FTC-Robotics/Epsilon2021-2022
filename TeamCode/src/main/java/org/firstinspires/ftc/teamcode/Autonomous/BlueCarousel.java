package org.firstinspires.ftc.teamcode.Autonomous;

import android.view.Surface;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import Epsilon.OurRobot;
import Epsilon.Subsystems.Drivetrain;
import Epsilon.Subsystems.OpenCV;
import Epsilon.Subsystems.Outtake;

@Autonomous
public class BlueCarousel extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        OurRobot.initialize(this);
        OurRobot.outtake.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OurRobot.outtake.upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake.PosASH scorePos = OpenCV.Pipeline.getAnalysis();
        //scan
        while (!isStarted()) {
            scorePos = OpenCV.Pipeline.getAnalysis();
            telemetry.addData("OpenCV Pos: ", scorePos);
            telemetry.update();
        }

        waitForStart();
        telemetry.addData("Encoder Counts ", OurRobot.outtake.upMotor.getCurrentPosition());
        telemetry.update();

        //drive to ASH
        OurRobot.drivetrain.Move(0.5, -25, Drivetrain.MoveType.STRAFE, this);
        //align with wall???
        OurRobot.drivetrain.Move(0.5, -16, Drivetrain.MoveType.DRIVE, this);

        OurRobot.outtake.scoreASH(scorePos);
        OurRobot.outtake.upMotor.setPower(0);

        //drive back, go to carousel
        OurRobot.drivetrain.Move(0.5, 7, Drivetrain.MoveType.DRIVE, this);

        //Strafe stuff down below
        OurRobot.imu.gyroTurn(0.7,0,this);
        OurRobot.drivetrain.Move(0.5, 60, Drivetrain.MoveType.STRAFE, this);
        //OurRobot.drivetrain.Move(0.5, 30, Drivetrain.MoveType.STRAFE, this);
        OurRobot.drivetrain.Move(0.3,2, Drivetrain.MoveType.DRIVE,this);

        OurRobot.carousel.duckMotor.setPower(0.6);
        OurRobot.wait(3000, this);
        OurRobot.carousel.duckMotor.setPower(0.0);

        OurRobot.drivetrain.Move(1, -20, Drivetrain.MoveType.DRIVE, this);

    }
}
