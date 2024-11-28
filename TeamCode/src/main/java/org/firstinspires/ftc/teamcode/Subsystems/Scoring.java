package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.Controls;


public class Scoring extends Mechanism {

    private Drivetrain drive = new Drivetrain(opMode);
    private OuttakeSlides slides = new OuttakeSlides(opMode);
    private Deposit deposit = new Deposit(opMode);
    private Intake intake;

    private int slidesPos = 0;
    public State state = State.INTAKE;
    private Intake.COLOR color;
    private boolean isBasket = false;
    private boolean intakeGrabbed = false;
    private boolean intakeOut = false;
    private boolean isClimb = false;
    private int LB, HB, LC, HC;

    public enum State {
        INTAKE,
        TRANSFER,
        SCORING,
        CLIMB
    }

    // Slide Commands
    private final Command slidesUp = () -> slides.setTarget(slidesPos);
    private final Command slidesIntake = () -> slides.intakePos();
    private final Command slideRest = () -> slides.setTarget(OuttakeSlides.REST_POS);
    private final Command lockSpecimen = () -> slides.lock();
    private final Command upABit = () -> slides.upABit();
    private final Command downABit = () -> slides.downABit();
    private final Command primeLevel1Ascent = () -> slides.primeAscent();
    private final Command Level1Ascent = () -> slides.ascent();
    // Deposit Commands
    private final Command depositPos = () -> deposit.depositPos();
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command basketPos = () -> deposit.basketPos();
    private final Command specimenSetPos = () -> deposit.specimenSetPos();
    private final Command specimenScorePos = () -> deposit.specimenScorePos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    private final Command specimenShift = () -> deposit.clawShift();
    // Intake Commands
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command dropV4b = () -> intake.barDown();
    private final Command neutralV4b = () -> intake.barNeutral();
    private final Command transferV4b = () -> intake.barTransfer();
    private final Command climbV4b = () -> intake.barFold();

    // Transfer Slide Command Sequences
    private CommandSequence depositTransferSequence = new CommandSequence()
            .addCommand(outtakeGrab)
            .addWaitCommand(.2)
            .addCommand(grabTransfer)
            .addCommand(upABit)
            .addWaitCommand(.5)
            .addCommand(outtakeRelease)
            .addCommand(slidesIntake)
            .build();
    private CommandSequence slidesDown = new CommandSequence()
            .addCommand(grabTransfer)
            .addCommand(outtakeRelease)
            .addCommand(slideRest)
            .build();
    private CommandSequence transeferRestPos = new CommandSequence()
            .addCommand(outtakeRelease)
            .addCommand(depositPos)
            .addCommand(slideRest)
            .build();
    private CommandSequence specimenPickUpSequence = new CommandSequence()
            .addCommand(slideRest)
            .addCommand(outtakeGrab)
            .addCommand(depositPos)
            .addWaitCommand(.4)
            .addCommand(outtakeRelease)
            .build();
    // Non-transfer Slide Command Sequences
    private CommandSequence ejectSampleSequence = new CommandSequence()
            .addCommand(outtakeGrab)
            .addCommand(slideRest)
            .addWaitCommand(.2)
            .addCommand(specimenScorePos)
            .addWaitCommand(0.4)
            .addCommand(outtakeRelease)
            .addWaitCommand(0.3)
            .addCommand(outtakeGrab)
            .addCommand(grabTransfer)
            .build();
    private CommandSequence depositSequence = new CommandSequence()
            .addCommand(outtakeGrab)
            .addWaitCommand(.5)
            .addCommand(slidesUp)
            .addWaitCommand(0.2)
            .addCommand(basketPos)
            .build();
    private CommandSequence depositSpecimenSequence = new CommandSequence()
            .addCommand(specimenShift)
            .addWaitCommand(.5)
            .addCommand(slidesUp)
            .addWaitCommand(0.2)
            .addCommand(outtakeGrab)
            .addCommand(specimenSetPos)
            .build();
    private CommandSequence depositSample = new CommandSequence()
            .addCommand(basketPos)
            .addCommand(outtakeRelease)
            .addWaitCommand(.3)
            .addCommand(grabTransfer)
            .addWaitCommand(.3)
            .addCommand(outtakeGrab)
            .addWaitCommand(.5)
            .addCommand(slideRest)
            .build();
    private CommandSequence depositSpecimen = new CommandSequence()
            .addCommand(outtakeGrab)
            .addCommand(specimenScorePos)
            .addWaitCommand(.4)
            .addCommand(lockSpecimen)
            .addWaitCommand(.4)
            .addCommand(outtakeRelease)
            .build();
    // Intake Command Sequences
    private CommandSequence primeIntakeSequence = new CommandSequence()
            .addCommand(dropV4b)
            .addWaitCommand(.5)
            .addCommand(intakeOpen)
            .build();
    private CommandSequence transferIntakeSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addWaitCommand(0.5)
            .addCommand(transferV4b)
            .addWaitCommand(1)
            .addCommand(intakeOpen)
            .addWaitCommand(.5)
            .addCommand(neutralV4b)
            .addWaitCommand(.1)
            .addCommand(intakeGrab)
            .build();
    // Ascend Command Sequences
    private CommandSequence primeAscent = new CommandSequence()
            .addCommand(basketPos)
            .addCommand(primeLevel1Ascent)
            .addWaitCommand(.5)
            .addCommand(climbV4b)
            .build();
    private CommandSequence ascend = new CommandSequence()
            .addCommand(Level1Ascent)
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
        deposit.depositPos();
        deposit.closeClaw();
    }

    public State getState() {
        return state;
    }

    public void resetSlide(){
        slides.downUntil();
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {
        drive.loop(gamepad1);
        slides.update();

        if (GamepadStatic.isButtonPressed(gamepad1.gamepad, Controls.CLIMB_SET)) state = State.CLIMB;

        switch (state){
            case INTAKE:

                if (GamepadStatic.wasJustPressed(gamepad2, Controls.PRIME_INTAKE)) {
                    if (intakeOut) {
                        transferIntakeSequence.trigger();
                        intakeOut = false;
                        state = State.TRANSFER;
                    } else {
                        primeIntakeSequence.trigger();
                        transeferRestPos.trigger();
                        intakeOut = true;
                    }
                } else if (GamepadStatic.wasJustPressed(gamepad2, Controls.GRAB)){
                    if (intakeGrabbed){
                        intakeOpen.run();
                    } else {
                        intakeGrab.run();
                    }
                }



                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_SPECIMEN)){
                    state = State.TRANSFER;
                    specimenPickUpSequence.trigger();
                }
                break;

            case TRANSFER:
                if (GamepadStatic.wasJustPressed(gamepad2 , Controls.PRIME_INTAKE)) {

                    primeIntakeSequence.trigger();

                    state = State.INTAKE;
                }

                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_TRANSFER)){

                    depositTransferSequence.trigger();
                } else if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_SPECIMEN)){

                    specimenPickUpSequence.trigger();
                }
                if(gamepad2.gamepad.right_trigger>0){

                    slides.setSlidePower(gamepad2.gamepad.right_trigger);

                } else if(gamepad2.gamepad.left_trigger>0){

                    slides.setSlidePower(-gamepad2.gamepad.left_trigger);

                } else {
                    for (int i = 0; i < 4; i++) {
                        if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.SLIDES[i])) {
                            state = State.SCORING;

                            slidesPos = OuttakeSlides.POSITIONS[i];
                            isBasket = OuttakeSlides.target == OuttakeSlides.HIGH_BASKET || OuttakeSlides.target == OuttakeSlides.LOW_BASKET;
                            if (isBasket) depositSequence.trigger();
                            else depositSpecimenSequence.trigger();
                        }
                    }
                }
                break;
            case SCORING:
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_TRANSFER)){

                    depositTransferSequence.trigger();

                    state = State.TRANSFER;
                } else if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_SPECIMEN)){

                    specimenPickUpSequence.trigger();

                    state = State.TRANSFER;
                }
                if(gamepad2.gamepad.right_trigger>0){

                    slides.setSlidePower(gamepad2.gamepad.right_trigger);

                } else if(gamepad2.gamepad.left_trigger>0){

                    slides.setSlidePower(-gamepad2.gamepad.left_trigger);

                } else {
                    for (int i = 0; i < 4; i++) {
                        if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.SLIDES[i])) {
                            slidesPos = OuttakeSlides.POSITIONS[i];
                            isBasket = i == 0 || i == 1;
                            if (isBasket) depositSequence.trigger();
                            else depositSpecimenSequence.trigger();
                        }
                    }
                }
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.RELEASE)) {
                    if (isBasket){
                        depositSample.trigger();
                    } else {
                        depositSpecimen.trigger();
                    }
                    state = State.TRANSFER;
                }
                break;
            case CLIMB:
                if (GamepadStatic.isButtonPressed(gamepad1.gamepad, Controls.CLIMB_SET)) {

                    primeAscent.trigger();
                    isClimb = true;
                } else if (isClimb && GamepadStatic.isButtonPressed(gamepad1.gamepad, Controls.CLIMB)){

                    ascend.trigger();
                }
                if (GamepadStatic.wasJustPressed(gamepad2 , Controls.PRIME_INTAKE)) {

                    primeIntakeSequence.trigger();
                    isClimb = false;

                    state = State.INTAKE;
                }
                for (int i = 0; i < 4; i++) {
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.SLIDES[i])) {
                        state = State.SCORING;
                    }
                }
                break;
        }
    }
}
