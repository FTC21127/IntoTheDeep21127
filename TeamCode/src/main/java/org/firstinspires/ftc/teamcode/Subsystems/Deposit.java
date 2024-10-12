package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.teleop.Controls;

/**Done(just tuning needed)*/

//added to allow tuning in ftc dashboard
@Config
public class Deposit extends Mechanism {

    //using ftclib ServoEx wrapper for extra functions.
    ServoEx claw, wrist1, wrist2;

    //Positions to be tuned
    public static double TRANSFER_POS = 0;
    public static double DEPOSIT_POS = 1;
    public static double SPECIMEN_POS = 0.8;
    public static double EJECT_SAMPLE = 0.7;
    public double GRAB = 0;
    public double RELEASE = 1;

    public Deposit(OpMode OpMode) {
        this.opMode = OpMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        wrist1 = new SimpleServo(hwMap,"wrist1", -20,90);
        wrist2 = new SimpleServo(hwMap,"wrist2", -90,20);
        claw = new SimpleServo(hwMap,"outtakeClaw", -5,40);
    }

    private void setPos(double pos){
        wrist1.setPosition(pos);
        wrist2.setPosition(pos-70);
    }

    public void depositPos(){
        setPos(DEPOSIT_POS);
    }

    public void transferPos(){
        setPos(TRANSFER_POS);
    }

    public void specimenPos(){
        setPos(SPECIMEN_POS);
    }

    public void eject(){
        setPos(EJECT_SAMPLE);
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
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.RELEASE)) {
            openClaw();
        }
    }
}