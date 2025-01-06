package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;

//Done
//added to allow tuning in ftc dashboard
@Config
public class Deposit extends Mechanism {

    //using ftclib ServoEx wrapper for extra functions.
    ServoEx claw, wrist1, wrist2;

    //Transfer: 0.9
    //Basket: 0.4
    //Specimen: 0.34
    //Specimen Score: 0.15
    //Deposit: 0.275
    public static double TRANSFER_POS = .97;
    public static double INIT_POS = .75;
    public static double STRAIGHT_UP_POS = .51;
    public static double DEPOSIT_POS = 0.16;
    public static double BASKET_POS = 0.3;
    public static double SPECIMEN_POS = 0.25;
    public static double SPECIMEN_SCORE_POS = 0.09;

//    public static double TRANSFER_POS = 0.07;
//    public static double DEPOSIT_POS = 0.37;
//    public static double BASKET_POS = 0.54;
//    public static double SPECIMEN_POS = 0.47;
//    public static double SPECIMEN_SCORE_POS = 0.27;
    public static double GRAB = 0.494;
    public static double SHIFT = 0.467;
    public static double RELEASE = .3;

    public Deposit(OpMode OpMode) {
        this.opMode = OpMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        wrist1 = new SimpleServo(hwMap,"wRight", 0,360);
        wrist2 = new SimpleServo(hwMap,"wLeft", 0,360);
        claw = new SimpleServo(hwMap,"outtakeClaw", 0,360);
    }

    public void setPos(double pos){
        wrist1.setPosition(1-pos);
        wrist2.setPosition(pos);
    }

    public void depositPos(){
        setPos(DEPOSIT_POS);
    }

    public void transferPos(){
        setPos(TRANSFER_POS);
    }

    public void initPos(){
        setPos(INIT_POS);
    }

    public void goofyBasketPos(){
        setPos(STRAIGHT_UP_POS);
    }

    public void basketPos(){
        setPos(BASKET_POS);
    }

    public void specimenSetPos(){
        setPos(SPECIMEN_POS);
    }

    public void specimenScorePos(){
        setPos(SPECIMEN_SCORE_POS);
    }

    public double getPos(){
        return wrist1.getPosition();
    }

    public void closeClaw(){
        claw.setPosition(GRAB);
    }

    public void openClaw(){
        claw.setPosition(RELEASE);
    }

    public void clawShift(){
        claw.setPosition(SHIFT);
    }

    @Override
    public void loop(FoozPad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.LEFT_BUMPER)) {
            openClaw();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.RIGHT_BUMPER)){
            closeClaw();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.DPAD_RIGHT)) {
            basketPos();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.DPAD_DOWN)) {
            transferPos();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.DPAD_UP)) {
            depositPos();
        }
        if (getPos()==DEPOSIT_POS){
            gamepad.assignedPad.rumble(2);
        }
    }
}