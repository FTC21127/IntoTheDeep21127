package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Controls;

/**Done(just tuning needed)*/

//added to allow tuning in ftc dashboard
@Config
public class Deposit extends Mechanism {

    //using ftclib ServoEx wrapper for extra functions.
    ServoEx claw, wrist1, wrist2;

    //Positions to be tuned
    public static double TRANSFER_POS = 1;
    public static double DEPOSIT_POS = 0;
    public static double BASKET_POS = 0.2;
    public double GRAB = 0;
    public double RELEASE = .375;

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
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.RELEASE)) {
            openClaw();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.GRAB)){
            closeClaw();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.DPAD_UP)) {
            basketPos();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.DPAD_DOWN)) {
            transferPos();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.LEFT_BUMPER)) {
            depositPos();
        }
    }
}