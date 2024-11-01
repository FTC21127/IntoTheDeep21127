package org.firstinspires.ftc.teamcode.opMode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp
public class Robot extends OpMode {
    Scoring bot = new Scoring(this, Intake.COLOR.RED);
    FoozPad gp1, gp2;

    @Override
    public void init() {
        bot.init(hardwareMap);
        gp1 = new FoozPad(gamepad1);
        gp2 = new FoozPad(gamepad2);
    }

    @Override
    public void loop() {
        bot.loop(gp1, gp2);
    }
}
