package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain2;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp(group = "dev")
public class Drive2Dev extends OpMode {
    Drivetrain2 drivetrain2 = new Drivetrain2(this);
    FoozPad gp1;
    @Override
    public void init() {
     drivetrain2.init(hardwareMap);
     gp1 = new FoozPad(gamepad1);
    }

    @Override
    public void loop() {
        gp1.update();
        drivetrain2.loop(gp1);
    }
}
