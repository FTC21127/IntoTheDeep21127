package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (group = "Testing")
public class MotorEncoderTesting extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        motor.setTargetPosition(2000);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (motor.isBusy()){
            motor.setPower(1);
        }

        motor.setTargetPosition(0);
        while (motor.isBusy()){
            motor.setPower(1);
        }

    }
}
