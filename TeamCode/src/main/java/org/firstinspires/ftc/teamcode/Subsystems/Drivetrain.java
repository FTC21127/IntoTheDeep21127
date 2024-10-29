package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Controls;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


// cargo vrooooom
public class Drivetrain extends Mechanism {

    // Use FTCLib's built in mecanum drivetrain class
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    // easier to reset heading than IMu, also counteracts imu drift
    double headingOffset = 0;

    public Drivetrain(OpMode OpMode) {
        this.opMode = OpMode;
    }

    // allows us to choose which type of driving we want
    public enum DRIVETYPE{
        FIELD,
        ROBOT
    }

    DRIVETYPE type = DRIVETYPE.ROBOT;
    HardwareMap hwMap;

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
        follower.setTeleOpMovementVectors(-gamepad.gamepad.left_stick_y, -gamepad.gamepad.left_stick_x, -gamepad.gamepad.right_stick_x);
        follower.update();
    }
}