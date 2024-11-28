package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Example extends OpMode {

    // Actual motor that runs
    DcMotor driveMotor;
    // Pseudo "motor" used to track position, don't set power to this one.
    DcMotor trackingEncoder;

    @Override
    public void init() {
        // Set the address of both motor objects to "exampleMotor"
        driveMotor = hardwareMap.get(DcMotor.class, "exampleMotor");
        trackingEncoder = hardwareMap.get(DcMotor.class, "exampleMotor");
    }

    @Override
    public void loop() {
        //run driveMotor however you want
        driveMotor.setPower(gamepad1.left_stick_y);
        // tracking motor's position is from the encoder port
        telemetry.addData("position: ", trackingEncoder.getCurrentPosition());
    }
}




