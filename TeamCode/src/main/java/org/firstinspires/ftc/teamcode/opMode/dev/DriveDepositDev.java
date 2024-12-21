package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp(group = "Dev")
public class DriveDepositDev extends OpMode {
    Drivetrain base = new Drivetrain(this);
    Deposit deposit = new Deposit(this);
    OuttakeSlides slides = new OuttakeSlides(this);
//    Intake intake = new Intake(this, Intake.COLOR.RED);
    FoozPad gp1, gp2;

    @Override
    public void init() {
        base.init(hardwareMap);
        deposit.init(hardwareMap);
        slides.init(hardwareMap);
//        intake.init(hardwareMap);
        slides.restPos();
        deposit.depositPos();
        gp1 = new FoozPad(gamepad1);
        gp2 = new FoozPad(gamepad2);
    }

    @Override
    public void loop() {
        gp2.update();
        gp1.update();
        base.loop(gp1);
        deposit.loop(gp2);
        slides.loop(gp2);
//        intake.loop(gp2);
        slides.telemetry(telemetry);
        telemetry.addData("gamepad: ",gp2.gamepad.a);
        telemetry.addData("previous: ",gp2.previous.a);
        telemetry.update();
    }
}
