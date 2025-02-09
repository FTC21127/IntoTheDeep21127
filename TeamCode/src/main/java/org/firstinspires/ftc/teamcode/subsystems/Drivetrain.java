package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.ControlsSemis;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;


// cargo vrooooom
public class Drivetrain extends Mechanism {

    private Follower follower;

    public Drivetrain(OpMode OpMode) {
        this.opMode = OpMode;
    }

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    @Override
    public void init(HardwareMap hwMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
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
        double y = -gamepad.gamepad.left_stick_y;
        double x = -gamepad.gamepad.left_stick_x;
        double r = -gamepad.gamepad.right_stick_x * .5;

        if (gamepad.gamepad.left_trigger>0.9){
            x *= -0.1;
            y *= -0.1;
            r = 0;
        } else {
            y = y * (1-gamepad.gamepad.left_trigger);
            x = x * (1-gamepad.gamepad.left_trigger);
            r = r * (1-gamepad.gamepad.left_trigger);
        }

        if (GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsSemis.SPIN_CLOCKWISE)){
            follower.setTeleOpMovementVectors(0, 0, 1);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsSemis.SPIN_COUNTER)){
            follower.setTeleOpMovementVectors(0, 0, -1);
        } else {
            follower.setTeleOpMovementVectors(y, x, r);
        }
        follower.update();
    }
}