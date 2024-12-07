package org.firstinspires.ftc.teamcode.opMode.dev;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "Dev")
public class DepositDev extends OpMode {
    Timing.Timer timer = new Timing.Timer(500, TimeUnit.SECONDS);
    Deposit outtake = new Deposit(this);
    FoozPad gp1;
    @Override
    public void init() {
        outtake.init(hardwareMap);
        gp1 = new FoozPad(gamepad1);
    }

    @Override
    public void loop() {
        outtake.loop(gp1);
        gp1.update();
    }
}
