package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Epsilon.OurRobot;
import Epsilon.Superclasses.EpsilonRobot;

@TeleOp
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        OurRobot robot = new OurRobot();    //creates instance of "OurRobot," giving it access to hardware/methods
        robot.initialize(this);

        waitForStart();

        while (opModeIsActive()){
           /*double y = -gamepad1.left_stick_y;
           double x = gamepad1.left_stick_x;
           double r = gamepad1.right_stick_x;

           robot.drivetrain.frontLeft.setPower(y+r+x);
           robot.drivetrain.frontRight.setPower(y-r-x);
           robot.drivetrain.backLeft.setPower(y+r-x);
           robot.drivetrain.backRight.setPower(y-r+x);
           */

           // OurRobot.imu.getError();


           //OurRobot.Odometry.update();
           //telemetry.addData("XPosition", OurRobot.Odometry.xPos);
           //telemetry.addData("YPosition", OurRobot.Odometry.yPos);
           //telemetry.addData("Angle", Math.toDegrees(OurRobot.Odometry.heading));
           // telemetry.update();
        }
    }
}
