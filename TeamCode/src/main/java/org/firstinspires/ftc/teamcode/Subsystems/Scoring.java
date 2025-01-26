package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.Controls;

import java.util.concurrent.TimeUnit;


public class Scoring extends Mechanism {

    Timing.Timer time = new Timing.Timer(2000, TimeUnit.MILLISECONDS);

    // Use subsystems already created
    public Drivetrain drive = new Drivetrain(opMode);
    public OuttakeSlides slides = new OuttakeSlides(opMode);
    private Deposit deposit = new Deposit(opMode);
    private Intake intake;

    private int slidesPos = 0; // Position to run the slides to
    public State state = State.TRANSFER; // State machine
    private Intake.COLOR color; // current alliance color (was never used)
    private HardwareMap hwMap;

    // failsafes
    private boolean isBasket = false;
    private boolean intakeGrabbed = false;
    private boolean intakeOut = false;
    private boolean isClimb = false;
    private boolean transferBucket = false;

    // different states of the robot
    public enum State {
        INTAKE,
        TRANSFER,
        SCORING,
        CLIMB
    }

    Command intakeExtend = this::mIntakeExtend;
    Command specimenSet = this::mSpecimenSet;
    Command basketSet = this::mBasketSet;
    Command depositRest = this::mSlideRest;

    // Slide Commands
    private final Command slidesUp = () -> slides.setTarget(slidesPos);
    private final Command slidesIntake = () -> slides.intakePos();
    private final Command lockSpecimen = () -> slides.lock();
    private final Command reset = slides::downUntil;
    // Deposit Commands
    private final Command depositPos = () -> deposit.depositPos();
    private final Command grabTransfer = this::mGrabSample;
    private final Command basketPos = () -> deposit.basketPos();
    private final Command specimenScorePos = () -> deposit.specimenScorePos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    private final Command specimenShift = () -> deposit.clawShift();
    // Intake Commands
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command pickUpV4b = () -> intake.barPickUp();
    private final Command neutralV4b = () -> intake.barNeutral();
    private final Command transferV4b = () -> intake.barTransfer();

    // Transfer Slide Command Sequences
    private CommandSequence depositTransferSequence = new CommandSequence() // sets the deposit ontop of the bucket
            .addCommand(grabTransfer)
            .addWaitCommand(.2)
            .addCommand(slidesIntake)
            .build();
    // Non-transfer Slide Command Sequences
    private CommandSequence resetSlide = new CommandSequence()
            .addCommand(reset)
            .build();

    private CommandSequence specimenSetSequence = new CommandSequence() // yeets sample out
            .addCommand(outtakeGrab)
            .addWaitCommand(.4)
            .addCommand(this::mSpecimenPickUp)
            .addWaitCommand(0.4)
            .addCommand(outtakeRelease)
            .build();
    private CommandSequence depositSequence = new CommandSequence() // grabs sample from bucket and goes to the set basket height
            .addCommand(outtakeGrab)
            .addWaitCommand(.4)
            .addCommand(basketSet)
            .build();
    private CommandSequence depositSpecimenSequence = new CommandSequence() // grabs specimen from wall and goes to chamber height
            .addCommand(specimenShift)
            .addWaitCommand(0.3)
            .addCommand(deposit::closeClaw)
            .addCommand(specimenSet)
            .build();
    private CommandSequence depositSample = new CommandSequence() // release sample and go down
            .addCommand(basketPos)
            .addWaitCommand(.15)
            .addCommand(outtakeRelease)
            .addWaitCommand(.2)
            .addCommand(depositRest)
            .build();
    private CommandSequence depositSpecimen = new CommandSequence() // lock specimen  and release
            .addCommand(outtakeGrab)
            .addCommand(specimenScorePos)
            .addWaitCommand(.4)
            .addCommand(lockSpecimen)
            .addWaitCommand(.3)
//            .addCommand(outtakeRelease)
            .addCommand(depositRest)
            .build();
    // Intake Command Sequences
    private CommandSequence primeIntakeSequence = new CommandSequence() // drop down the intake
            .addCommand(intakeExtend)
            .addWaitCommand(.2)
            .addCommand(intakeOpen)
            .addCommand(outtakeRelease)
            .build();
    private CommandSequence retractIntake = new CommandSequence() // pickup the sample and put into the bucket
            .addCommand(intakeGrab)
            .addCommand(pickUpV4b)
            .addWaitCommand(.4)
            .addCommand(transferV4b)
            .addWaitCommand(1)
            .addCommand(intakeOpen)
            .addWaitCommand(.4)
            .addCommand(neutralV4b)
            .addWaitCommand(.1)
            .addCommand(intakeGrab)
            .build();

    // Constructor to input parameter
    public Scoring(OpMode opMode, Intake.COLOR color) {
        this.opMode = opMode;
        this.color = color;
    }

    @Override
    public void init(HardwareMap hwMap) {
        // init everything and hardware map stuff
        intake = new Intake(opMode, color);
        drive.init(hwMap);
        slides.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);
        this.hwMap = hwMap;
    }

    //Gets the current robot state
    public State getState() {
        return state;
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {
        drive.loop(gamepad1);
        drive.setTp(1); // Sets turn speed to 100%

        // Slide reset
        if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.RESET)) {
            resetSlide.trigger();
        }

        if (slides.reset) {

            // Manual slide control
            if (gamepad2.gamepad.right_trigger > 0) {
                slides.setSlidePower(gamepad2.gamepad.right_trigger);
            } else if (gamepad2.gamepad.left_trigger > 0) {
                slides.setSlidePower(-.2);
            } else {
                slides.update();
            }

            // Quick switch to CLIMB state
            if (GamepadStatic.isButtonPressed(gamepad1.gamepad, Controls.CLIMB_SET)) {
                state = State.CLIMB;
                mClimbSet();
            }

            // When left bumper is held, intake is extended
            if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.PRIME_INTAKE)) {
                if (state != State.INTAKE) {
                    primeIntakeSequence.trigger();
                    state = State.INTAKE;
                }
            } else if (state == State.INTAKE) {
                retractIntake.trigger();
                state = State.TRANSFER;
            }

            switch (state) {
                case INTAKE:
                    drive.setTp(0.77); // Sets turn speed to 80%
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_ELEMENT)||GamepadStatic.isButtonPressed(gamepad2.gamepad, GamepadStatic.Input.DPAD_LEFT)) {
                        if (transferBucket) {
                            specimenSetSequence.trigger();
                            transferBucket = false;
                        }
                        else {
                            depositTransferSequence.trigger();
                            transferBucket = true;
                        }
                        state = State.TRANSFER;
                    }
                    break;

                case TRANSFER:
                    if (GamepadStatic.wasJustPressed(gamepad2, Controls.GRAB_SPECIMEN)||GamepadStatic.isButtonPressed(gamepad2.gamepad, GamepadStatic.Input.DPAD_LEFT)) {
                        if (transferBucket) {
                            specimenSetSequence.trigger();
                            transferBucket = false;
                        }
                        else {
                            depositTransferSequence.trigger();
                            transferBucket = true;
                        }
                    }
                    for (int i = 0; i < 4; i++) {
                        if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.SLIDES[i])) {
                            state = State.SCORING;

                            slidesPos = OuttakeSlides.POSITIONS[i];
                            isBasket = OuttakeSlides.target == OuttakeSlides.HIGH_BASKET || OuttakeSlides.target == OuttakeSlides.LOW_BASKET;
                            if (isBasket) depositSequence.trigger();
                            else depositSpecimenSequence.trigger();
                        }
                    }
                    break;
                case SCORING:
                    drive.setTp(0.77); // Sets turn speed to 80%

                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, Controls.GRAB_ELEMENT)||GamepadStatic.isButtonPressed(gamepad2.gamepad, GamepadStatic.Input.DPAD_LEFT)) {
                            transferBucket = false;
                            mSpecimenPickUp();
                        state = State.TRANSFER;
                        break;
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
                        if (isBasket) {
                            depositSample.trigger();
                        } else {
                            depositSpecimen.trigger();
                        }
                        state = State.TRANSFER;
                    }
                    break;
                case CLIMB:
                    deposit.initPos();
                    intake.closeClaw();
                    intake.barFold();
                    if (GamepadStatic.isButtonPressed(gamepad1.gamepad, Controls.CLIMB_SET)) {
                        mClimbSet();
                        isClimb = true;
                    } else if (isClimb && GamepadStatic.isButtonPressed(gamepad1.gamepad, Controls.CLIMB)) {

                        slides.ascent();
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

    public void initM() {
        intake.barNeutral();
        deposit.goofyBasketPos();
        slides.reset();
        slides.restPos();
    }

    public void mIntakeExtend() {
        intake.setV4B(Intake.BAR_DOWN+0.02);
        mSpecimenPickUp();
    }

    public void mSpecimenPickUp() {
        slides.restPos();
        deposit.depositPos();
        deposit.openClaw();
    }

    public void mGrabSample() {
        slides.restPos();
        deposit.transferPos();
        deposit.openClaw();
    }

    public void mSpecimenSet() {
        slides.setTarget(slidesPos);
        intake.barNeutral();
        deposit.specimenSetPos();
        deposit.closeClaw();
    }

    public void mBasketSet() {
        slides.setTarget(slidesPos);
        intake.barNeutral();
        deposit.goofyBasketPos();
    }

    public void mSlideRest() {
        slides.intakePos();
        deposit.openClaw();
        deposit.goofyBasketPos();
    }

    public void mClimbSet() {
        slides.primeAscent();
        deposit.initPos();
        intake.closeClaw();
        intake.barFold();
    }
}