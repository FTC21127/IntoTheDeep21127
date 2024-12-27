package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain2;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;

@TeleOp(group = "dev")
public class Drive2Dev extends OpMode {
    Drivetrain2 drivetrain2 = new Drivetrain2(this);
    FoozPad gp1;
    IMU imu;

    double desiredHeading = 0, offSet = 0;

    @Override
    public void init() {
     drivetrain2.init(hardwareMap);
     gp1 = new FoozPad(gamepad1);
     imu = hardwareMap.get(IMU.class, "imu");
     imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
     imu.resetYaw();
    }

    @Override
    public void loop() {
        telemetry.addData("Desired Heading:", desiredHeading);
        telemetry.addData("OffSet Heading:", offSet);
        telemetry.addData("Yaw:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("pitch:", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("roll:", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));

        desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (gamepad1.right_stick_x != 0 || GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.X) || GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.B)){
            desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } else if (GamepadStatic.wasJustPressed(gp1, GamepadStatic.Input.A)){
            desiredHeading-= 180;
        }

        offSet = desiredHeading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        drivetrain2.setOffSet(offSet);

        gp1.update();
        drivetrain2.loop(gp1);
    }
}
