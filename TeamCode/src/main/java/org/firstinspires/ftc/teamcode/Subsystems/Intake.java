package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Controls;


@Config
public class Intake extends Mechanism {

    ServoEx horizontalExtendenator;
    ServoEx v4b;
    ServoEx claw;
    NormalizedColorSensor color; // give color values in range from 0 - 1

    //Positions to be tuned
    public static double BAR_DOWN = 0.0525;
    public static double BAR_TRANSFER = 0.67;
    public static double BAR_NEUTRAL = 0.5;
    public static double BAR_FOLD = 0.7;
    public static double GRIP = .075;
    public static double RELEASE = .35;
    public static double SLIDE_COMPRESS = 0;
    public static double SLIDE_NEUTRAL = .5;

    COLOR alliance;

    public enum COLOR{
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public Intake(OpMode opMode1){
        this(opMode1,COLOR.RED);
    }

    public Intake(OpMode opMode1, COLOR alliance) {
        this.opMode = opMode1;
        this.alliance = alliance;
    }

    @Override
    public void init(HardwareMap hwMap) {
//        color = hwMap.get(NormalizedColorSensor.class, "color");
        claw = new SimpleServo(hwMap,"intakeClaw", 0,40);
        v4b = new SimpleServo(hwMap,"intakeArm",-20,120);
//        horizontalExtendenator = new SimpleServo(hwMap, "intakeSlides", 0, 110);

//        color.setGain(35);
    }

    public void closeClaw(){
        claw.setPosition(GRIP);
    }

    public void openClaw(){
        claw.setPosition(RELEASE);
    }

    public void barDown(){
        v4b.setPosition(BAR_DOWN);
    }

    public void barNeutral(){
        v4b.setPosition(BAR_NEUTRAL);
    }

    public void barTransfer(){
        v4b.setPosition(BAR_TRANSFER);
    }

    public void barFold(){
        v4b.setPosition(BAR_FOLD);
    }

    public void setSlidePos(double pos){
        horizontalExtendenator.setPosition(pos);
    }

    public void extendSlide(){
        horizontalExtendenator.setPosition(SLIDE_NEUTRAL);
    }

    public void retractSlide(){
        horizontalExtendenator.setPosition(SLIDE_COMPRESS);
    }

    public double getV4bPos() {
        return v4b.getPosition();
    }

    public boolean isTransferPos(){
        return getV4bPos()==BAR_TRANSFER;
    }

    @Override
    public void loop(FoozPad gamepad) {
        if (gamepad.gamepad.right_trigger > 0.1){
            closeClaw();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.PRIME_INTAKE)) {
            barDown();
        } else if (gamepad.gamepad.left_trigger > 0.2) {
            openClaw();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.START)){
            barTransfer();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.BACK)) {
            barNeutral();
        }
    }

    public COLOR sampleColor(){
        if (color.getNormalizedColors().green > 0.5 && color.getNormalizedColors().red > .4) return COLOR.YELLOW;
        if (color.getNormalizedColors().red > 0.4 && color.getNormalizedColors().blue < 0.4) return COLOR.RED;
        if (color.getNormalizedColors().blue > 0.4) return COLOR.BLUE;
        return COLOR.NONE;
    }

}
