package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


// cargo vrooooom
public class Drivetrain2 extends Mechanism {

    // Use FTCLib's built in mecanum drivetrain class
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    public Drivetrain2(OpMode OpMode, DRIVETYPE drivetype) {
        this.type = drivetype;
        this.opMode = OpMode;
    }

    // allows us to choose which type of driving we want
    public enum DRIVETYPE{
        FIELD(false),
        ROBOT(true);
        boolean isRobot;

        DRIVETYPE(boolean isRobot) {
            this.isRobot = isRobot;
        }
    }

    DRIVETYPE type = DRIVETYPE.ROBOT;

    @Override
    public void init(HardwareMap hwMap) {
        follower = new Follower(hwMap);

        leftFront = hwMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hwMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hwMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hwMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();
    }

    @Override
    public void loop(FoozPad gamepad) {
        gamepad.update();
        double y = -gamepad.gamepad.left_stick_y / 2;
        double x = -gamepad.gamepad.left_stick_x * .4;
        double r = -gamepad.gamepad.right_stick_x * .4;

        y = y * (1+gamepad.gamepad.right_trigger*.4) * (1-gamepad.gamepad.left_trigger);
        x = x * (1+gamepad.gamepad.right_trigger*.4) * (1-gamepad.gamepad.left_trigger);
        r = r * (1+gamepad.gamepad.right_trigger*.4) * (1-gamepad.gamepad.left_trigger);

        follower.setTeleOpMovementVectors(y, x, r, type.isRobot);
        follower.update();
    }
}