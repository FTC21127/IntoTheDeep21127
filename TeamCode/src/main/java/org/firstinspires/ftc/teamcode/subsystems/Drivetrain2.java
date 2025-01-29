package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.ControlsSemis;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

// cargo vrooooom
public class Drivetrain2 extends Mechanism {

    private Follower follower;
    private PIDController headingController = new PIDController(
            2,
            0,
            0.1
    );
    private IMU imu;

    double desiredHeading = 0, currentHeading = 0;

    public Drivetrain2(OpMode OpMode) {
        this.opMode = OpMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hwMap);

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        follower.startTeleopDrive();
        imu.resetYaw();
        currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        desiredHeading = currentHeading;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Desired Heading:", desiredHeading);
        telemetry.addData("Current Heading:", currentHeading);
        telemetry.addData("Yaw:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("pitch:", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
    }

    @Override
    public void loop(FoozPad gamepad) {
        gamepad.update();
        currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.X)) {
            follower.setTeleOpMovementVectors(0, 0, 1);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.B)) {
            follower.setTeleOpMovementVectors(0, 0, -1);
        } else {


            double y = -gamepad.gamepad.left_stick_y * .8;
            double x = -gamepad.gamepad.left_stick_x * .65;
            double r = -gamepad.gamepad.right_stick_x * .5;

            y = y * (1 + gamepad.gamepad.right_trigger * .25) * (1 - gamepad.gamepad.left_trigger);
            x = x * (1 + gamepad.gamepad.right_trigger * .25) * (1 - gamepad.gamepad.left_trigger);
            r = r * (1 - gamepad.gamepad.left_trigger);

            if (r == 0) {
                follower.setTeleOpMovementVectors(y, x, headingController.calculate(currentHeading, desiredHeading));
            } else {
                desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                follower.setTeleOpMovementVectors(y, x, r);
            }
        }

        if (GamepadStatic.wasJustPressed(gamepad, ControlsSemis.FLIP)){
            headingController.setSetPoint(desiredHeading-180);
        }
        follower.update();
    }
}