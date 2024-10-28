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
    private Intake intake;

    private int slidesPos = 0;
    private State state = State.INTAKE;
    private Intake.COLOR color;

    private enum State {
        INTAKE,
        TRANSFER,
        SCORING
    }

    private Command slidesUp = () -> slides.setTarget(slidesPos);
    private Command slidesIntake = () -> slides.setTarget(OuttakeSlides.INTAKE_POS);
    private Command depositPos = () -> deposit.depositPos();
    private Command grabTransfer = () -> deposit.transferPos();
    private Command outtakeRelease = () -> deposit.openClaw();
    private Command outtakeGrab = () -> deposit.closeClaw();
    private Command eject = () -> deposit.eject();
    private Command intakeGrab = () -> intake.closeClaw();
    private Command intakeOpen = () -> intake.openClaw();
    private Command dropV4b = () -> intake.barDown();
    private Command neutralV4b = () -> intake.barNeutral();
    private Command transferV4b = () -> intake.barTransfer();
    private Command slideRest = () -> slides.restPos();

    private CommandSequence pickUpTransfer = new CommandSequence()
            .addCommand(grabTransfer)
            .addCommand(slidesIntake)
            .build();
    private CommandSequence ejectSample = new CommandSequence()
            .addCommand(eject)
            .addWaitCommand(0.4)
            .addCommand(outtakeRelease)
            .addWaitCommand(0.3)
            .addCommand(grabTransfer)
            .build();
    private CommandSequence depositSequence = new CommandSequence()
            .addCommand(outtakeGrab)
            .addCommand(slidesUp)
            .addCommand(depositPos)
            .build();
    private CommandSequence intakeSequence = new CommandSequence()
            .addCommand(dropV4b)
            .addWaitCommand(.5)
            .addCommand(intakeOpen)
            .build();
    private CommandSequence grabSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addWaitCommand(0.1)
            .addCommand(transferV4b)
            .addWaitCommand(0.4)
            .addCommand(intakeOpen)
            .addWaitCommand(0.6)
            .addCommand(neutralV4b)
            .build();


    public Scoring(OpMode opMode, Intake.COLOR color) {
        this.opMode = opMode;
        this.color = color;
    }

    @Override
    public void init(HardwareMap hwMap) {
        intake = new Intake(opMode,color);
        drive.init(hwMap);
        slides.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);
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
