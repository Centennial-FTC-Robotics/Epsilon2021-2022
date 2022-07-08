package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Epsilon.OurRobot;
import Epsilon.Subsystems.Drivetrain;
import Epsilon.Subsystems.OpenCV;
import Epsilon.Subsystems.Outtake;

@Autonomous
public class RedCarousel extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        OurRobot.initialize(this);
        Outtake.PosASH scorePos = Outtake.PosASH.TOP;
        //scan
        while (!isStarted()) {
            scorePos = OpenCV.Pipeline.getAnalysis();
            telemetry.addData("OpenCV Pos: ", scorePos);
            telemetry.update();
        }

        waitForStart();

        //drive to ASH
        OurRobot.drivetrain.Move(0.5, 25, Drivetrain.MoveType.STRAFE, this);
        OurRobot.drivetrain.Move(0.5, -16, Drivetrain.MoveType.DRIVE, this);

        //score
        OurRobot.outtake.scoreASH(scorePos);

        //OurRobot.CycleFreight(this);

        OurRobot.drivetrain.Move(0.5, 12, Drivetrain.MoveType.DRIVE, this);

        //drive back, go to carousel
        //OurRobot.imu.gyroTurn(0.8, 90, this);
        OurRobot.drivetrain.Move(0.5, -17, Drivetrain.MoveType.TURN, this);
        OurRobot.drivetrain.Move(0.35, 45, Drivetrain.MoveType.DRIVE, this);
        OurRobot.carousel.duckMotor.setPower(-0.5);
        OurRobot.wait(5000, this);
        OurRobot.carousel.duckMotor.setPower(0.0);
        OurRobot.drivetrain.Move(0.5, -25, Drivetrain.MoveType.STRAFE, this);
    }
}