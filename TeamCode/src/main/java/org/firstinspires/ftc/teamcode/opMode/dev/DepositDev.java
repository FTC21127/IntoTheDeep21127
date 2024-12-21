package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp(group = "Dev")
public class DepositDev extends OpMode {
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
