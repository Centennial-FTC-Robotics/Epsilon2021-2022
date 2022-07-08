package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Epsilon.OurRobot;
import Epsilon.Subsystems.Outtake;
import Epsilon.Superclasses.EpsilonRobot;

@TeleOp
public class TestFinalTeleOp extends LinearOpMode {
    public enum OuttakeState{
        OUTTAKE_INIT,
        VERTICAL_EXTEND,
        HORIZONTAL_EXTEND,
        DOOR_OPEN,
        DOOR_CLOSE,
        VERTICAL_RETRACT,
        HORIZONTAL_RETRACT,
        OUTTAKE_END
    }
    @Override
    public void runOpMode() throws InterruptedException{
        //OurRobot robot = new OurRobot();    //creates instance of "OurRobot," giving it access to hardware/methods
        OurRobot.initialize(this);
        OurRobot.drivetrain.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OurRobot.drivetrain.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OurRobot.drivetrain.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OurRobot.drivetrain.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double verticalPosition = OurRobot.outtake.upMotor.getCurrentPosition();

        waitForStart();

        double speed = -0.7;
        double outtakeFSMSpeed = 0.6;


        ElapsedTime time = new ElapsedTime();
        boolean verticalActive = false;
        boolean horizontalActive = false;

        boolean doorToggle = true;
        boolean lastBumperState = false;

        double outtakeInitTime = time.milliseconds();
        double wheelSpeed = 0.9;

        OuttakeState outtakeState = OuttakeState.OUTTAKE_INIT;

        int level = OurRobot.outtake.FLOOR;

        while (opModeIsActive()){

            telemetry.addData("X", OurRobot.Odometry.encoderX.getCurrentPosition());
            telemetry.addData("Y", OurRobot.Odometry.encoderY.getCurrentPosition());
            /*****************
             * Drive Formula *
             ****************/
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double r = -gamepad1.right_stick_x;
            /*
            OurRobot.drivetrain.frontLeft.setPower(speed*(y-r-x));
            OurRobot.drivetrain.frontRight.setPower(speed*(y+r+x));
            OurRobot.drivetrain.backLeft.setPower(speed*(y-r+x));
            OurRobot.drivetrain.backRight.setPower(speed*(y+r-x));
            */
            if (gamepad1.right_trigger>0)           //slowmode
                speed = 0.3;
            else if (gamepad1.left_trigger>0)       //fastmode
                speed = 1.0;
            else
                speed = 0.7;

            /**********
             * Intake *
             **********/
            //if(gamepad1.right_bumper)
                //wheelSpeed = 0.75;

            if(gamepad1.left_bumper)
                OurRobot.intake.wheel.setPower(-wheelSpeed);
            else if (gamepad1.right_bumper) {
                wheelSpeed = 0.75;
                OurRobot.intake.wheel.setPower(wheelSpeed);
            }else
                OurRobot.intake.wheel.setPower(0.0);


            /********************************
             * Outtake Finite State Machine *
             *******************************/
            switch (outtakeState){
                case OUTTAKE_INIT:

                    //dpad control of the linear slides
                    if (gamepad2.dpad_up) {
                        OurRobot.outtake.upMotor.setPower(0.85);
                        verticalPosition = OurRobot.outtake.upMotor.getCurrentPosition();
                    } else if (gamepad2.dpad_down) {
                        OurRobot.outtake.upMotor.setPower(-0.3);
                        verticalPosition = OurRobot.outtake.upMotor.getCurrentPosition();
                    } else
                        OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(verticalPosition));

                    //dpad control of the horizontal servo
                    if (gamepad2.dpad_left) {
                        OurRobot.outtake.arm.setPosition(0.9);
                    } else if (gamepad2.dpad_right) {
                        OurRobot.outtake.arm.setPosition(0);
                    }

                    //automation
                    /*
                    if (gamepad2.x) {
                        level = OurRobot.outtake.ASH_TOP;
                        outtakeInitTime = time.milliseconds();
                        outtakeState = OuttakeState.VERTICAL_EXTEND;
                    }
                    else if (gamepad2.a) {
                        level = OurRobot.outtake.ASH_MID;
                        outtakeInitTime = time.milliseconds();
                        outtakeState = OuttakeState.VERTICAL_EXTEND;
                    } else if (gamepad2.y) {
                        level = OurRobot.outtake.ASH_BOTTOM;
                        outtakeInitTime = time.milliseconds();
                        outtakeState = OuttakeState.VERTICAL_EXTEND;
                    }
                    */
                    break;
                case VERTICAL_EXTEND:
                        OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(level));
                        outtakeState = OuttakeState.HORIZONTAL_EXTEND;
                    break;
                case HORIZONTAL_EXTEND:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(level));
                    if(time.milliseconds() > outtakeInitTime + 2000) {
                        OurRobot.outtake.setHorizontal(OurRobot.outtake.ARM_EXTEND);
                        outtakeState = OuttakeState.DOOR_OPEN;
                    }
                    break;
                case DOOR_OPEN:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(level));
                    if(time.milliseconds() > outtakeInitTime + 4000) {
                        OurRobot.outtake.openDoor();
                        outtakeState = OuttakeState.DOOR_CLOSE;
                    }
                    break;
                case DOOR_CLOSE:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(level));
                    if(time.milliseconds() > outtakeInitTime + 6000) {
                        OurRobot.outtake.closeDoor();
                        outtakeState = OuttakeState.HORIZONTAL_RETRACT;
                    }
                    break;
                case HORIZONTAL_RETRACT:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(level));
                    if(time.milliseconds() > outtakeInitTime + 8000) {
                        OurRobot.outtake.setHorizontal(OurRobot.outtake.ARM_RETRACT);
                        outtakeState = OuttakeState.VERTICAL_RETRACT;
                    }
                    break;
                case VERTICAL_RETRACT:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(level));
                    if(time.milliseconds() > outtakeInitTime + 10000) {
                        level = OurRobot.outtake.FLOOR;
                        outtakeState = OuttakeState.OUTTAKE_END;
                    }
                    break;
                case OUTTAKE_END:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(level));
                    if(time.milliseconds() > outtakeInitTime + 12000) {
                        verticalPosition = OurRobot.outtake.upMotor.getCurrentPosition();
                        outtakeState = OuttakeState.OUTTAKE_INIT;
                    }
                    break;
                default:
                    outtakeState = OuttakeState.OUTTAKE_INIT;
            }
            //elemetry.addData("level",level);
            //telemetry.addData("outtake fsm",outtakeState);

            /*******************
             * Door and Toggle *
             ******************/
            /*
            boolean currentRightBumper = gamepad2.right_bumper;
            //If statement to confirm that the bumper really changed values between while loops
            if (currentRightBumper && !lastBumperState){
                doorToggle = !doorToggle;
            }
            lastBumperState = currentRightBumper;

            if (doorToggle) {
                OurRobot.outtake.closeDoor();
            } else if(!doorToggle) {
                OurRobot.outtake.openDoor();
            }
            */
            if(outtakeState==OuttakeState.OUTTAKE_INIT) {
                if (gamepad2.right_bumper)
                    OurRobot.outtake.openDoor();
                else
                    OurRobot.outtake.closeDoor();
            }

            /************
             * Carousel *
             ***********/

            if(gamepad1.x)
                OurRobot.carousel.duckMotor.setPower(0.5);
            else if(gamepad1.y)
                OurRobot.carousel.duckMotor.setPower(-0.5);
            else
                OurRobot.carousel.duckMotor.setPower(0.0);

            /***********
             * Capper *
             **********/

            OurRobot.capper.tapeExtender.setPower(gamepad2.left_trigger);
            OurRobot.capper.tapeExtender.setPower(-gamepad2.right_trigger);

            OurRobot.capper.pivot.setPower(-0.75 * gamepad2.right_stick_y);
            
            OurRobot.capper.tapeRotator.setPower(0.4*gamepad2.right_stick_x);
            /*
            if (gamepad2.a)
                OurRobot.capper.pivot.setPower(0.7);
            else if (gamepad2.y)
                OurRobot.capper.pivot.setPower(-0.7);
            else
                OurRobot.capper.pivot.setPower(0.0);
               */



            //telemetry.addData("Door Toggle", doorToggle);
            //telemetry.addData("Horizontal",OurRobot.outtake.arm.getPosition());
            //telemetry.addData("Vertical",OurRobot.outtake.upMotor.getCurrentPosition());
            telemetry.update();


          /*  OurRobot.Odometry.update();
            telemetry.addData("XPosition", OurRobot.Odometry.xPos);
            telemetry.addData("YPosition", OurRobot.Odometry.yPos);
            telemetry.addData("Angle", Math.toDegrees(OurRobot.Odometry.heading));
            telemetry.update(); */
        }
    }


}
