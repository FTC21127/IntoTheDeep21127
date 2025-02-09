package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Climb;

@TeleOp(group = "Dev")
public class ClimbDev extends OpMode {
    Climb climb = new Climb(this);
    @Override
    public void init() {
        climb.init(hardwareMap);
    }

    @Override
    public void loop() {
        climb.lockServo();
    }
}
