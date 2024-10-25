package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;

public class Scoring extends Mechanism {

    private Drivetrain drive = new Drivetrain(opMode);
    private OuttakeSlides slides = new OuttakeSlides(opMode);
    private Deposit deposit = new Deposit(opMode);

    private int slidesPos = 0;
    private State state = State.INTAKE;

    private enum State {
        INTAKE,
        TRANSFER,
        SCORING
    }

    private Command slidesUp = () -> slides.setTarget(slidesPos);
    private Command slidesIntake = () -> slides.setTarget(OuttakeSlides.INTAKE_POS);
    private Command depositPos = () -> deposit.depositPos();
    private Command grabTransfer = () -> deposit.transferPos();
    private Command clawOpen = () -> deposit.openClaw();
    private Command clawClose = () -> deposit.closeClaw();
    private Command eject = () -> deposit.eject();

    private CommandSequence pickUpTransfer = new CommandSequence()
            .addCommand(grabTransfer)
            .addCommand(slidesIntake)
            .addCommand(clawClose)
            .build();
    private CommandSequence ejectSample = new CommandSequence()
            .addCommand(eject)
            .addCommand(clawOpen).addWaitCommand(0.5)
            .addCommand(grabTransfer)
            .build();
    private CommandSequence depositSample = new CommandSequence()
            .addCommand(slidesUp)
            .addCommand(depositPos)
            .build();

    public Scoring(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        drive.init(hwMap);
        slides.init(hwMap);
        deposit.init(hwMap);
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {
        drive.loop(gamepad1);
        slides.update();

        switch (state){
            case INTAKE:

                break;

            case SCORING:

                break;

            case TRANSFER:

                break;
        }
    }
}
