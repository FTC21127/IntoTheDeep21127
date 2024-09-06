package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Mecanum Drive TeleOp", group="TeleOp")
public class Tele extends OpMode {

    // Declare motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        // Initialize motors from the hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set the direction of the motors so the robot moves correctly
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Get the gamepad values for driving the robot
        double y = -gamepad1.left_stick_y;  // Remember, the gamepad y-axis is reversed
        double x = gamepad1.left_stick_x * 1.1;  // Adjusted for imperfect strafing
        double rotation = gamepad1.right_stick_x;

        // Calculate the power for each motor using the Mecanum drive formula
        double frontLeftPower = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        // Normalize the values so no power exceeds 1.0


        // Set the power to the motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Telemetry for debugging
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when the op mode is stopped
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}