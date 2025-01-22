package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;

public class Test extends OpMode {

    Deposit deposit = new Deposit(this);
    FoozPad foozPad;

    @Override
    public void init() {
        deposit.init(hardwareMap);
        foozPad = new FoozPad(gamepad1);


    }

    @Override
    public void loop() {

    }
}
