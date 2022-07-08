package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Epsilon.OurRobot;
import Epsilon.Subsystems.Drivetrain;
import Epsilon.Subsystems.OpenCV;
import Epsilon.Subsystems.Outtake;

@Autonomous
public class RedWarehouse extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        OurRobot.initialize(this);

        waitForStart();

        //for just driving into warehouse
        //OurRobot.drivetrain.Move(0.5,55, Drivetrain.MoveType.DRIVE, this);

        Outtake.PosASH scorePos = OpenCV.Pipeline.getAnalysis();

        //Strafes left to shipping hub, approaches it
        OurRobot.drivetrain.Move(0.5, -25, Drivetrain.MoveType.STRAFE, this);
        OurRobot.drivetrain.Move(0.5, -19, Drivetrain.MoveType.DRIVE, this);

        //Scores ASH
        OurRobot.outtake.scoreASH(scorePos);

        //Drives back, turns 90, drives to warehouse
        OurRobot.drivetrain.Move(0.5, 5, Drivetrain.MoveType.DRIVE, this);
        OurRobot.drivetrain.Move(0.3, -18, Drivetrain.MoveType.TURN, this);

        //Drive into warehouse
        OurRobot.drivetrain.Move(0.7,-70, Drivetrain.MoveType.DRIVE, this);
    }

}
