package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.teleop.Controls;


// cargo vrooooom
public class Drivetrain extends Mechanism {

    // Use FTCLib's built in mecanum drivetrain class
    Motor m1, m2 , m3, m4;
    MecanumDrive base;
    RevIMU imu;
    // easier to reset heading than IMu, also counteracts imu drift
    double headingOffset = 0;

    public Drivetrain(OpMode OpMode) {
        this.opMode = OpMode;
    }

    // allows us to choose which type of driving we want
    enum DRIVETYPE{
        FIELD,
        ROBOT
    }

    DRIVETYPE type = DRIVETYPE.ROBOT;

    @Override
    public void init(HardwareMap hwMap) {
        // init of stuff
        m1 = new Motor(hwMap, "frontLeft");
        m2 = new Motor(hwMap, "frontRight");
        m3 = new Motor(hwMap, "backLeft");
        m4 = new Motor(hwMap, "backRight");

        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        base = new MecanumDrive(m1, m2, m3, m4);

        imu  = new RevIMU(hwMap, "imu");
        imu.init();
    }

    public void driveType(DRIVETYPE type){
        this.type = type;
    }

    @Override
    public void loop(Gamepad gamepad) {
        // reset heading
        if (GamepadStatic.isButtonPressed(gamepad, Controls.RESET_HEADING1)&&GamepadStatic.isButtonPressed(gamepad, Controls.RESET_HEADING2)){
            setHeading(0);
        }

        switch (type) {
            case ROBOT: // robot centric
                base.driveRobotCentric(gamepad.left_stick_x, gamepad.left_stick_y, -gamepad.right_stick_x);
                break;
            case FIELD: // field centric
                base.driveFieldCentric(gamepad.left_stick_x, gamepad.left_stick_y, -gamepad.right_stick_x, getHeading());
                break;
        }
    }

    public void setHeading(double h) {
        headingOffset = h - getIMUHeading();
    }

    public double getHeading() {
        return headingOffset + getIMUHeading();
    }

    public double getIMUHeading() {
        return imu.getRotation2d().getDegrees();
    }
}