package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (group = "Testing")
public class MotorEncoderTesting extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(5000);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(1);

        while (motor.isBusy()){}

        motor.setTargetPosition(-5000);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motor.isBusy()){}

        motor.setPower(0);
    }
}
