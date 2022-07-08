package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Epsilon.OurRobot;
import Epsilon.Subsystems.Drivetrain;
import Epsilon.Subsystems.OpenCV;
import Epsilon.Subsystems.Outtake;

@Autonomous
public class BlueWarehouse extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        OurRobot.initialize(this);

        Outtake.PosASH scorePos = OpenCV.Pipeline.getAnalysis();
        telemetry.addData("OpenCV Pos: ", scorePos);
        telemetry.update();

        waitForStart();

        noCycle(scorePos);
        //Cycle();
        //cycle();

    }

    public void Cycle(){
        //drive to ASH
        OurRobot.drivetrain.Move(0.5, 25, Drivetrain.MoveType.STRAFE, this);
        OurRobot.drivetrain.Move(0.5, -19, Drivetrain.MoveType.DRIVE, this);

        //score
        OurRobot.outtake.scoreASH(Outtake.PosASH.TOP);

        //drive back and turn
        OurRobot.drivetrain.Move(0.5, 16, Drivetrain.MoveType.DRIVE, this);

        OurRobot.drivetrain.Move(0.3, -18, Drivetrain.MoveType.TURN, this);
        OurRobot.drivetrain.Move(0.3, -18, Drivetrain.MoveType.TURN, this);
        OurRobot.drivetrain.Move(0.3, -18, Drivetrain.MoveType.TURN, this);

        OurRobot.drivetrain.Move(0.5, -10, Drivetrain.MoveType.STRAFE, this);

        OurRobot.CycleFreight(this);
        OurRobot.CycleFreight(this);

        OurRobot.drivetrain.Move(0.75, 20, Drivetrain.MoveType.DRIVE, this);
    }

    public void noCycle(Outtake.PosASH scorePos){
        //drive to ASH
        OurRobot.drivetrain.Move(0.5, 25, Drivetrain.MoveType.STRAFE, this);
        OurRobot.drivetrain.Move(0.5, -19, Drivetrain.MoveType.DRIVE, this);

        //score ASH
        OurRobot.outtake.scoreASH(scorePos);

        //Move back and turn
        OurRobot.drivetrain.Move(0.5, 5, Drivetrain.MoveType.DRIVE, this);
        OurRobot.drivetrain.Move(0.3, 18, Drivetrain.MoveType.TURN, this);

        //Drive into warehouse
        OurRobot.drivetrain.Move(0.7,-70, Drivetrain.MoveType.DRIVE, this);
    }

    public void cycle(){
        driveToWarehouse();
        intakeElement();
        clearIntake();
        returnToASH();
        score(Outtake.PosASH.TOP);
    }

    public void driveToWarehouse() {
        followPath(false);

    }

    public void returnToASH(){
        followPath(true);
    }

    public void followPath(boolean reverse){

    }

    public void intakeElement(){
        OurRobot.intake.wheel.setPower(1.0);
    }

    public void clearIntake(){
        OurRobot.intake.wheel.setPower(-1.0);
    }

    public void score(Outtake.PosASH scorePos){
        OurRobot.outtake.extend(OurRobot.outtake.ASH_TOP);
    }

    public void drive(int inches, double power){
        OurRobot.drivetrain.Move(0.5,-55, Drivetrain.MoveType.DRIVE, this);
    }

    public void turn90(){
        OurRobot.drivetrain.Move(0.3, 18, Drivetrain.MoveType.TURN, this);
    }
}
