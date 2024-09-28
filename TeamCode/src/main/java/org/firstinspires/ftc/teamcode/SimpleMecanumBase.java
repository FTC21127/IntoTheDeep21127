package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SimpleMecanumBase extends OpMode {
    Motor m1, m2 , m3, m4;
    MecanumDrive base;
    GamepadEx gp1;
    RevIMU imu;
    boolean fieldCentric = false;
    ButtonReader baseCentric, resetIMU;

    @Override
    public void init() {
        m1 = new Motor(hardwareMap, "frontLeft");
        m2 = new Motor(hardwareMap, "frontRight");
        m3 = new Motor(hardwareMap, "backLeft");
        m4 = new Motor(hardwareMap, "backRight");

        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        base = new MecanumDrive(m1, m2, m3, m4);
        gp1 = new GamepadEx(gamepad1);
        imu  = new RevIMU(hardwareMap, "imu");
        imu.init();
        baseCentric = new ButtonReader(gp1, GamepadKeys.Button.A);
        resetIMU = new ButtonReader(gp1, GamepadKeys.Button.DPAD_DOWN);
    }

    @Override
    public void loop() {
        if (resetIMU.wasJustPressed()){
            imu.reset();    
        }
        if (baseCentric.wasJustPressed())
            fieldCentric = !fieldCentric;
        if (!fieldCentric) {
            base.driveRobotCentric(gp1.getLeftX(), gp1.getLeftY(), -gp1.getRightX());
        } else {
            base.driveFieldCentric(gp1.getLeftX(), gp1.getLeftY(), -gp1.getRightX(), imu.getRotation2d().getDegrees());
        }
        telemetry.addData("Robot heading", imu.getRotation2d());
    }
}
