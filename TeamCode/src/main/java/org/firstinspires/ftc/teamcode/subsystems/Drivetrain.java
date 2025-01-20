package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.ControlsM3;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;


// cargo vrooooom
public class Drivetrain extends Mechanism {

    private Follower follower;

    public Drivetrain(OpMode OpMode) {
        this.opMode = OpMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hwMap);
        follower.startTeleopDrive();
    }

    @Override
    public void loop(FoozPad gamepad) {
        gamepad.update();
        double y = -gamepad.gamepad.left_stick_y * .8;
        double x = -gamepad.gamepad.left_stick_x * .65;
        double r = -gamepad.gamepad.right_stick_x * .5;

        y = y * (1+gamepad.gamepad.right_trigger*.25) * (1-gamepad.gamepad.left_trigger);
        x = x * (1+gamepad.gamepad.right_trigger*.25) * (1-gamepad.gamepad.left_trigger);
        r = r * (1-gamepad.gamepad.left_trigger);

        if (GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsM3.SPIN_CLOCKWISE)){
            follower.setTeleOpMovementVectors(0, 0, 1);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsM3.SPIN_COUNTER)){
            follower.setTeleOpMovementVectors(0, 0, -1);
        } else {
            follower.setTeleOpMovementVectors(y, x, r);
        }
        follower.update();
    }
}