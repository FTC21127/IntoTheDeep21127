package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Controls;


public class Scoring extends Mechanism {

    private Drivetrain drive = new Drivetrain(opMode);
    private OuttakeSlides slides = new OuttakeSlides(opMode);
    private Deposit deposit = new Deposit(opMode);
    private Intake intake;

    private int slidesPos = 0;
    private State state = State.INTAKE;
    private Intake.COLOR color;
    private boolean isBarTransfer = false;
    private boolean isBasket = false;
    private boolean intakeGrabbed = false;
    private int LB, HB, LC, HC;

    private enum State {
        INTAKE,
        TRANSFER,
        SCORING
    }

    private final Command slidesUp = () -> slides.setTarget(slidesPos);
    private final Command slidesIntake = () -> slides.intakePos();
    private final Command slideRest = () -> slides.restPos();
    private final Command lockSpecimen = () -> slides.lock();
    private final Command depositPos = () -> deposit.depositPos();
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command basketPos = () -> deposit.basketPos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command dropV4b = () -> intake.barDown();
    private final Command neutralV4b = () -> intake.barNeutral();
    private final Command transferV4b = () -> intake.barTransfer();

    private CommandSequence depositTransferSequence = new CommandSequence()
            .addCommand(grabTransfer)
            .addCommand(slidesIntake)
            .build();
    private CommandSequence specimenPickUpSequence = new CommandSequence()
            .addCommand(depositPos)
            .addCommand(slideRest)
            .build();
    private CommandSequence ejectSampleSequence = new CommandSequence()
            .addCommand(depositPos)
            .addWaitCommand(0.4)
            .addCommand(outtakeRelease)
            .addWaitCommand(0.3)
            .addCommand(grabTransfer)
            .build();
    private CommandSequence depositSequence = new CommandSequence()
            .addCommand(outtakeGrab)
            .addWaitCommand(.5)
            .addCommand(slidesUp)
            .addWaitCommand(0.2)
            .addCommand(basketPos)
            .build();
    private CommandSequence depositSample = new CommandSequence()
            .addCommand(depositPos)
            .addCommand(outtakeRelease)
            .build();
    private CommandSequence depositSpecimen = new CommandSequence()
            .addCommand(basketPos)
            .addWaitCommand(.5)
            .addCommand(outtakeRelease)
            .build();
    private CommandSequence primeIntakeSequence = new CommandSequence()
            .addCommand(dropV4b)
            .addCommand(intakeOpen)
            .build();
    private CommandSequence transferIntakeSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addWaitCommand(0.3)
            .addCommand(transferV4b)
            .addWaitCommand(1)
            .addCommand(intakeOpen)
            .addWaitCommand(0.6)
            .addCommand(neutralV4b)
            .addCommand(intakeGrab)
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
        intake.barNeutral();
        deposit.transferPos();
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {
        drive.loop(gamepad1);
        slides.update();

        switch (state){
            case INTAKE:
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.PRIME_INTAKE)){
                    primeIntakeSequence.trigger();
                } else if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB)){
                    specimenPickUpSequence.trigger();
                    transferIntakeSequence.trigger();
                    state = State.TRANSFER;
                }
                break;

            case TRANSFER:
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad , Controls.PRIME_INTAKE)) {state = State.INTAKE; break;}


                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_TRANSFER)){
                    depositTransferSequence.trigger();
                } else if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_SPECIMEN)){
                    specimenPickUpSequence.trigger();
                }
                for (int i = 0; i < 4; i++) {
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.SLIDES[i])){
                        state = State.SCORING;
                        slidesPos = OuttakeSlides.POSITIONS[i];
                        depositSequence.trigger();
                    }
                }
                break;
            case SCORING:
                for (int i = 0; i < 4; i++) {
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.SLIDES[i])){
                        slidesPos = OuttakeSlides.POSITIONS[i];
                        isBasket = OuttakeSlides.target == OuttakeSlides.HIGH_BASKET || OuttakeSlides.target == OuttakeSlides.LOW_BASKET;
                        depositSequence.trigger();
                    }
                }
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.RELEASE)) {
                    if (isBasket){
                        depositSample.trigger();
                    } else {
                        depositSpecimen.trigger();
                    }
                    specimenPickUpSequence.trigger();
                }
                break;
        }

    }
}
