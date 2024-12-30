package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.ControlsM3;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


// cargo vrooooom
public class Drivetrain2 extends Mechanism {

    private Follower follower;
    private PIDController headingController = new PIDController(
            1.5,
            0,
            0.2
    );
    private IMU imu;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    double desiredHeading = 0;

    public Drivetrain2(OpMode OpMode) {
        this.opMode = OpMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        follower = new Follower(hwMap);

        leftFront = hwMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hwMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hwMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hwMap.get(DcMotorEx.class, rightFrontMotorName);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();

        desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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

        if (gamepad.gamepad.right_stick_x == 0 && !GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsM3.SPIN_COUNTER) && !GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsM3.SPIN_CLOCKWISE)){
            follower.setTeleOpMovementVectors(y, x, -headingController.calculate());
        } else {
            desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            headingController.setSetPoint(desiredHeading);
            follower.setTeleOpMovementVectors(y, x,r);
        }
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsM3.SPIN_CLOCKWISE)) {
            follower.setTeleOpMovementVectors(0, 0, 1);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsM3.SPIN_COUNTER)) {
            follower.setTeleOpMovementVectors(0, 0, -1);
        }
        if (GamepadStatic.wasJustPressed(gamepad, ControlsM3.FLIP)){
            headingController.setSetPoint(desiredHeading-180);
        }
        follower.update();
    }
}