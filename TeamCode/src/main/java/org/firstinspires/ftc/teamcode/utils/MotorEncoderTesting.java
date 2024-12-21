package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp (group = "Testing")
public class MotorEncoderTesting extends LinearOpMode {
    DcMotorEx motor;
    MotionProfiler testProfile = new MotionProfiler(3,0.3,0.5);

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(2000);

        waitForStart();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testProfile.setTargetPos(motor.getCurrentPosition(),0,0);
        while (Math.abs(motor.getCurrentPosition()) > testProfile.offsetDist){
            motor.setVelocity(testProfile.getTargetVel(motor.getCurrentPosition()));
            telemetry.addData("Pos: ", motor.getVelocity());
            telemetry.update();
        }
    }
}
