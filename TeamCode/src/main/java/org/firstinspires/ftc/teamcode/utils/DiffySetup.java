package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(group = "Testing")
public class DiffySetup extends OpMode {

    Servo diffyLeft, diffyRight;

    public enum DiffyState{
        DOWN,
        TRANSFER,
        SEARCH,
        NEUTRAL,
    }

    Intake.DiffyState intakeState = Intake.DiffyState.NEUTRAL;

    public static double dif_TRANSFER = 0.7;
    public static double dif_INTERPOSED = .4;
    public static double dif_DOWN = 0.1;
    public static double dif_SEARCH = 0.2;
    public static double dif_ROLL = 0;
    public double dif_PITCH = dif_TRANSFER;

    public void setDif_ROLL(double degrees) {
        degrees /= 90;
        Intake.dif_ROLL = Math.min(Math.max(-.1,degrees),.1);
    }

    public void diffyDown(){
        dif_PITCH = dif_DOWN;
        intakeState = Intake.DiffyState.DOWN;
    }

    public void diffySearch(){
        dif_PITCH = dif_SEARCH;
        intakeState = Intake.DiffyState.SEARCH;
    }

    public void diffyTransfer(){
        dif_PITCH = dif_TRANSFER;
        dif_ROLL = 0;
        intakeState = Intake.DiffyState.TRANSFER;
    }

    public void diffyInterposed(){
        dif_PITCH = dif_INTERPOSED;
        dif_ROLL = 0;
        intakeState = Intake.DiffyState.NEUTRAL;
    }

    @Override
    public void init() {
        diffyLeft = hardwareMap.get(Servo.class, "diffyLeft");
        diffyRight = hardwareMap.get(Servo.class, "diffyRight");
    }

    @Override
    public void loop() {
            dif_ROLL = gamepad1.right_stick_x/10;
            diffyRight.setPosition(1 - dif_PITCH + dif_ROLL);
            diffyLeft.setPosition(dif_PITCH + dif_ROLL);
            if(gamepad1.cross){
                diffyDown();
            } else if (gamepad1.circle) {
                diffySearch();
            } else if (gamepad1.triangle) {
                diffyInterposed();
            } else if (gamepad1.square) {
                diffyTransfer();
            }
    }
}
