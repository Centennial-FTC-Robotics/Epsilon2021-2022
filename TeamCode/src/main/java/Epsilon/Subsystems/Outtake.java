package Epsilon.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static Epsilon.Subsystems.Outtake.OuttakeState.VERTICAL_EXTEND;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeleOp.TestFinalTeleOp;

import Epsilon.OurRobot;
import Epsilon.Superclasses.Subsystem;

//Initializes all the motors/hardware for the drivetrain
public class Outtake implements Subsystem {

    public final double TICKS_PER_ROTATION = 357.7;       //this is correct
    double circumference = 1.8897637795;                             //circumference in inches
    public final double TICKS_PER_INCH = TICKS_PER_ROTATION/circumference;      //temporary numbers lmao please fix later
    public final double ARM_EXTEND = 0.9;
    public final double ARM_RETRACT = 0.0;
    public final int ASH_BOTTOM = 400; //930
    public final int ASH_MID = 740; //1450
    public final int ASH_TOP = 1150; //1785
    public final int FLOOR = 130;

    public DcMotor upMotor;
    public Servo arm;
    public Servo door;

    public void initialize(LinearOpMode opMode) {
        upMotor = opMode.hardwareMap.dcMotor.get("top");
        arm = opMode.hardwareMap.servo.get("bottom");
        door = opMode.hardwareMap.servo.get("door");

        upMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double PID(double targetPos) {
        double kP = 0.01;
        double power;
        double error = targetPos - upMotor.getCurrentPosition();
        power = error * kP;
        return power;
    }

    public void extendo(double position, int verticalTicks, double power){
        upMotor.setTargetPosition(verticalTicks);
        upMotor.setPower(power);
        arm.setPosition(position);
    }
    public void openDoor(){
        door.setPosition(1.0);
    }

    public void closeDoor(){
        door.setPosition(0.0);
    }

    public void reset(){
        arm.setPosition(0.0);
        upMotor.setTargetPosition(350);
        upMotor.setPower(-0.3);
    }
    public void setVertical(double power, int ticks){
        power = Range.clip(power, -0.3, 0.6);
        upMotor.setPower(power);
        upMotor.setTargetPosition(ticks);
    }
    public void setHorizontal(double position) {
        position = Range.clip(position, 0, 1);
        arm.setPosition(position);
    }
    public enum PosASH {
        TOP,
        MID,
        BOTTOM
    }

    public void scoreASH(PosASH pos){
        if (pos == PosASH.BOTTOM) {
            extend(ASH_BOTTOM);
        } else if (pos == PosASH.MID) {
            extend(ASH_MID);
        } else if (pos == PosASH.TOP){
            extend(ASH_TOP);
        }
        //reset();
    }

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

    public void extend(int ashLevel){
        ElapsedTime time = new ElapsedTime();
        boolean complete = false;

        OuttakeState outtakeState = OuttakeState.OUTTAKE_INIT;

        double outtakeInitTime = time.milliseconds();

        while(!complete){
            switch (outtakeState){
                case OUTTAKE_INIT:
                    outtakeState = VERTICAL_EXTEND;
                    break;
                case VERTICAL_EXTEND:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(ashLevel));
                    outtakeState = OuttakeState.HORIZONTAL_EXTEND;
                    break;
                case HORIZONTAL_EXTEND:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(ashLevel));
                    if(time.milliseconds() > outtakeInitTime + 1000) {
                        OurRobot.outtake.setHorizontal(OurRobot.outtake.ARM_EXTEND);
                        outtakeState = OuttakeState.DOOR_OPEN;
                    }
                    break;
                case DOOR_OPEN:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(ashLevel));
                    if(time.milliseconds() > outtakeInitTime + 2000) {
                        OurRobot.outtake.openDoor();
                        outtakeState = OuttakeState.DOOR_CLOSE;
                    }
                    break;
                case DOOR_CLOSE:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(ashLevel));
                    if(time.milliseconds() > outtakeInitTime + 3000) {
                        OurRobot.outtake.closeDoor();
                        outtakeState = OuttakeState.HORIZONTAL_RETRACT;
                    }
                    break;
                case HORIZONTAL_RETRACT:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(ashLevel));
                    if(time.milliseconds() > outtakeInitTime + 4000) {
                        OurRobot.outtake.setHorizontal(OurRobot.outtake.ARM_RETRACT);
                        outtakeState = OuttakeState.VERTICAL_RETRACT;
                    }
                    break;
                case VERTICAL_RETRACT:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(ashLevel));
                    if(time.milliseconds() > outtakeInitTime + 5500) {
                        outtakeState = OuttakeState.OUTTAKE_END;
                    }
                    break;
                case OUTTAKE_END:
                    OurRobot.outtake.upMotor.setPower(OurRobot.outtake.PID(FLOOR));
                    if(time.milliseconds() > outtakeInitTime + 6500) {
                        complete = true;
                    }
                    break;
                default:
                    outtakeState = OuttakeState.OUTTAKE_INIT;
            }
        }
    }

}
