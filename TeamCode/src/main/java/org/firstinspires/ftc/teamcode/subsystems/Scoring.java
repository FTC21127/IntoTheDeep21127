package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.auton.utils.Colors;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.ControlsM3;

public class Scoring extends Mechanism {

    private final Drivetrain drive = new Drivetrain(opMode);
    private final OuttakeSlides slides = new OuttakeSlides(opMode);
    private final Deposit deposit = new Deposit(opMode);
    private Intake diffy = new Intake(opMode);

    private int slidesPos = 0;
    private State state = State.INTAKE;
    private final Colors color;
    private boolean isBasket = false;
    private boolean isTransfer = false;

    public enum State {
        INTAKE,
        TRANSFER,
        SCORING,
        CLIMB
    }

    //Intake commands
    Command diffyDown = () -> {diffy.diffyDown();};
    Command diffyInterpose = () -> {diffy.diffyInterposed();};
    Command diffyGrab = () -> {diffy.closeClaw();};
    Command diffyRetract = () -> {diffy.retractExtendy();};
    Command diffyTransfer = () -> {diffy.diffyTransfer();};
    Command difftShift = () -> {diffy.shiftClaw();};
    // Deposit commands
    Command depositGrab = deposit::closeClaw;
    Command depositRelease = deposit::openClaw;

    CommandSequence yeetSample = new CommandSequence()
            .addCommand(diffy::extendNeutral)
            .addCommand(diffyInterpose)
            .addWaitCommand(.5)
            .addCommand(diffy::openClaw)
            .addWaitCommand(.2)
            .addCommand(diffyRetract)
            .addCommand(diffyTransfer)
            .build();

    CommandSequence retractIntake = new CommandSequence()
            .addCommand(diffyDown)
            .addCommand(this::mSlideRest)
            .addWaitCommand(.2)
            .addCommand(difftShift)
            .addWaitCommand(.2)
            .addCommand(diffyInterpose)
            .addCommand(diffyRetract)
            .addWaitCommand(.2)
            .addCommand(diffyGrab)
            .addWaitCommand(.1)
            .addCommand(diffyTransfer)
            .build();
    CommandSequence basketSet = new CommandSequence()
            .addCommand(depositGrab)
            .addWaitCommand(.5)
            .addCommand(diffy::openClaw)
            .addCommand(this::mBasketSet)
            .build();
    CommandSequence specimenSet = new CommandSequence()
            .addCommand(depositGrab)
            .addWaitCommand(.2)
            .addCommand(diffy::openClaw)
            .addCommand(this::mSpecimenSet)
            .build();
    CommandSequence basketRelease = new CommandSequence()
            .addCommand(depositRelease)
            .addWaitCommand(.3)
            .addCommand(this::mSlideRest)
            .build();
    CommandSequence specimenRelease = new CommandSequence()
            .addCommand(depositRelease)
            .build();


    public Scoring(OpMode opMode, Colors color) {
        this.opMode = opMode;
        this.color = color;
    }

    public State getState() {
        return state;
    }

    @Override
    public void init(HardwareMap hwMap) {
        drive.init(hwMap);
        slides.init(hwMap);
        deposit.init(hwMap);
        diffy.init(hwMap);
        diffy.setAlliance(color);
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {
        slides.update();

        if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.INTAKE)) {
            if (state != State.INTAKE) {
                mIntakeExtend();
                state = State.INTAKE;
            }
        } else if (state == State.INTAKE) {
            retractIntake.trigger();
            isTransfer = true;
            state = State.TRANSFER;
        }
        if (GamepadStatic.isButtonPressed(gamepad1.gamepad, ControlsM3.CLIMB_SET)) {
            mClimbSet();
            state = State.CLIMB;
        }

        switch (state) {
            case INTAKE:
                diffy.teleControl(gamepad2);
                break;
            case TRANSFER:
                for (int i = 0; i < 4; i++) {
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.SLIDES[i])) {
                        state = State.SCORING;
                        isTransfer = false;
                        slidesPos = OuttakeSlides.POSITIONS[i];
                        isBasket = i == 1 || i == 0;
                        if (isBasket) basketSet.trigger();
                        else specimenSet.trigger();
                        break;
                    }
                }
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.SPECIMEN_POS)) {
                    if (isTransfer){
                        yeetSample.trigger();
                    } else {
                        mSpecimenPickUp();
                    }
                    isTransfer = false;
                }

                break;
            case SCORING:
                for (int i = 0; i < 4; i++) {
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.SLIDES[i])) {
                        slidesPos = OuttakeSlides.POSITIONS[i];
                        isBasket = i == 1 || i == 0;
                        if (isBasket) basketSet.trigger();
                        else specimenSet.trigger();
                    }
                }
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.RELEASE)) {
                    if (isBasket) basketRelease.trigger();
                    else specimenRelease.trigger();
                    state = State.TRANSFER;
                }
                break;
            case CLIMB:
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.CLIMB)) {
                    slides.ascent();
                    diffyRetract.run();
                    diffyTransfer.run();
                }
                break;
        }


    }

    public void mIntakeExtend() {
        diffy.extendNeutral();
        diffy.diffyDown();
        diffy.openClaw();
        mSlideRest();
    }

    public void mSpecimenPickUp() {
        slides.restPos();
        deposit.specimenGrabPos();
        deposit.openClaw();
    }

    public void mSpecimenSet() {
        slides.setTarget(slidesPos);
        diffy.diffyInterposed();
        deposit.specimenPos();
    }

    public void mBasketSet() {
        slides.setTarget(slidesPos);
        diffy.diffyInterposed();
        deposit.basketPos();
    }

    public void mSlideRest() {
        slides.intakePos();
        deposit.openClaw();
        deposit.transferPos();
    }

    public void mClimbSet() {
        slides.primeAscent();
        deposit.initPos();
    }

    public void stop(){
        diffy.retractExtendy();
        deposit.transferPos();
    }

}
