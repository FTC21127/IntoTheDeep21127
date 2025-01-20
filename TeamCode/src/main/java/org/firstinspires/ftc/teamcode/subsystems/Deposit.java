package org.firstinspires.ftc.teamcode.subsystems;

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
    public static double TRANSFER_POS = 0;
    public static double INIT_POS = .25;
    public static double STRAIGHT_UP_POS = .3;
    public static double BASKET_POS = 0.5;
    public static double SPECIMEN_SCORE_POS = 0.6;
    public static double SPECIMEN_SET_POS = .53;
    public static double SPECIMEN_GRAB_POS = 0.6;

    public static double GRAB = 0.38;
    public static double RELEASE = 0.2;

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

    public void initPos(){
        setPos(INIT_POS);
    }

    public void transferPos(){
        setPos(TRANSFER_POS);
    }

    public void basketPos(){
        setPos(BASKET_POS);
    }

    public void specimenPos(){
        setPos(SPECIMEN_SET_POS);
    }

    public void specimenScorePos(){
        setPos(SPECIMEN_SCORE_POS);
    }

    public void specimenGrabPos(){
        setPos(SPECIMEN_GRAB_POS);
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
            initPos();
        }
        if (getPos()==INIT_POS){
            gamepad.gamepad.rumble(2);
        }
    }
}