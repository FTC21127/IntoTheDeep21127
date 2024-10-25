package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp(group = "Dev")
public class DriveDepositDev extends OpMode {
    Drivetrain base = new Drivetrain(this);
    Deposit deposit = new Deposit(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    FoozPad gp1, gp2;

    @Override
    public void init() {
        base.init(hardwareMap);
        deposit.init(hardwareMap);
        slides.init(hardwareMap);
        slides.restPos();
        deposit.depositPos();
        gp1 = new FoozPad(gamepad1);
        gp2 = new FoozPad(gamepad2);
    }

    @Override
    public void loop() {
        base.loop(gp1);
        deposit.loop(gp2);
        slides.loop(gp2);
    }
}
