package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (group = "Testing")
public class Example extends OpMode {

   Servo servo;

    @Override
    public void init() {
      servo = hardwareMap.get(Servo.class, "horizontalExtendi");
    }

    @Override
    public void loop() {
        if(gamepad1.cross){
            servo.setPosition(0);
        } else if (gamepad1.circle) {
            servo.setPosition(.75);
        }
    }
}




