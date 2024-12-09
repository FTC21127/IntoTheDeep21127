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

    //Positions to be tuned
    public static double TRANSFER_POS = 0.05;
    public static double DEPOSIT_POS = 0.38;
    public static double BASKET_POS = 0.55;
    public static double SPECIMEN_POS = 0.40;
    public static double SPECIMEN_SCORE_POS = 0.25;
    public static double GRAB = 0.05;
    public static double SHIFT = 0.1;
    public static double RELEASE = .315;

    public Deposit(OpMode OpMode) {
        this.opMode = OpMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        wrist1 = new SimpleServo(hwMap,"wRight", 0,90);
        wrist2 = new SimpleServo(hwMap,"wLeft", 0,90);
        claw = new SimpleServo(hwMap,"outtakeClaw", -5,40);
    }

    private void setPos(double pos){
        wrist1.setPosition(pos);
        wrist2.setPosition(1-pos);
    }

    public void depositPos(){
        setPos(DEPOSIT_POS);
    }

    public void transferPos(){
        setPos(TRANSFER_POS);
    }

    public void basketPos(){
        setPos(BASKET_POS);
    }

    public void specimenPos(){
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
            gamepad.gamepad.rumble(2);
        }
    }
}