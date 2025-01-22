package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain2;

@TeleOp(group = "Dev")
public class DrivetrainDev extends OpMode {
    Drivetrain2 drive = new Drivetrain2(this);
    FoozPad gp1;

    @Override
    public void init() {
        drive.init(hardwareMap);
        gp1 = new FoozPad(gamepad1);
    }

    @Override
    public void loop() {
        gp1.update();
        drive.loop(gp1);
    }
}
