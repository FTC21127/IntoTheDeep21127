package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.auton.utils.Colors;
import org.firstinspires.ftc.teamcode.opMode.teleop.ControlsM3;


public class Scoring extends Mechanism {

    private Drivetrain drive = new Drivetrain(opMode);
    private OuttakeSlides slides = new OuttakeSlides(opMode);
    private Deposit deposit = new Deposit(opMode);
    private Intake diffy;

    private int slidesPos = 0;
    private State state = State.INTAKE;
    private Colors color;
    private boolean isBasket = false;
    private boolean diffyGrabbed = false;
    private boolean diffyOut = false;
    private boolean isClimb = false;
    private int LB, HB, LC, HC;

    private enum State {
        INTAKE,
        TRANSFER,
        SCORING,
        CLIMB
    }

    // Slide Commands

    // Deposit Commands

    // Intake Commands

    // Transfer Slide Command Sequences

    // Non-transfer Slide Command Sequences

    // Intake Command Sequences

    // Ascend Command Sequences



    public Scoring(OpMode opMode, Colors color) {
        this.opMode = opMode;
        this.color = color;
    }

    @Override
    public void init(HardwareMap hwMap) {
        diffy = new Intake(opMode,color);
        drive.init(hwMap);
        slides.init(hwMap);
        deposit.init(hwMap);
        diffy.init(hwMap);
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {
        slides.update();

        if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.INTAKE)){
            if (state != State.INTAKE){
                mIntakeExtend();
                state = State.INTAKE;
            }
        } else if (state == State.INTAKE) {
            //TODO: add retract seqeunce
            state = State.TRANSFER;
        }
        if (GamepadStatic.isButtonPressed(gamepad1.gamepad, ControlsM3.CLIMB_SET)){
            state = State.CLIMB;
        }

        switch (state){
            case INTAKE:
                diffy.teleControl(gamepad2);
                break;
            case TRANSFER:
                for (int i = 0; i < 4; i++) {
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.SLIDES[i])) {
                        state = State.SCORING;
                        slidesPos = OuttakeSlides.POSITIONS[i];
                        isBasket = i==1 || i==0;
                        if (isBasket); //TODO: put in commmand sequences
                        else ;
                    }
                }
                if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.SPECIMEN_POS)){
                    mSpecimenPickUp();
                }
                break;
            case SCORING:
                for (int i = 0; i < 4; i++) {
                    if (GamepadStatic.isButtonPressed(gamepad2.gamepad, ControlsM3.SLIDES[i])) {
                        state = State.SCORING;
                        slidesPos = OuttakeSlides.POSITIONS[i];
                        isBasket = i==1 || i==0;
                        if (isBasket); //TODO: put in commmand initialization
                        else ;
                    }
                }
                break;
        }


    }

    public void mIntakeExtend(){
        diffy.extendNeutral();
        diffy.diffySearch();
        diffy.openClaw();
    }

    public void mSpecimenSet(){
        slides.setTarget(slidesPos);
        deposit.specimenPos();
    }

    public void mBasketSet(){
        slides.setTarget(slidesPos);
        deposit.basketPos();
    }

    public void mSpecimenPickUp(){
        slides.restPos(); // or something, idk rlly
        deposit.specimenGrabPos();
        deposit.openClaw();
    }

}