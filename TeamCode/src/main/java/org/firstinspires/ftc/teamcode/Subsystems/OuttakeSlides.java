package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.Controls;

import java.util.concurrent.TimeUnit;

// Done
@Config   // @Config here is just gonna be used for easy tuning via FTC Dashboard
public class OuttakeSlides extends Mechanism {

    // This is the outtake slides class.
    // Includes a PID controller, set positions, and manual controls

    MotorEx slideR, slideL;

    //Use voltage sensor
    private VoltageSensor voltage;
    Timing.Timer time = new Timing.Timer(2000, TimeUnit.MILLISECONDS);

    // PID controller coefficients
    private final double p = 0.0175, i = 0, d = 0.0005, f = 0;

    // Positions for slides
    public static int REST_POS = 100;
    public static int INTAKE_POS = 30;
    public static int LOW_BASKET = 2000; //0
    public static int HIGH_BASKET = 3800; //1
    public static int LOW_CHAMBER_SET = 400; //2
    public static int HIGH_CHAMBER_SET = 1500; //3
    public static int CHAMBER_SCORED = 400;
    public static int LEVEL_1_ASCENT = 2000;
    public static int HANG = 0;
    public static int ABIT = 100;

    public static double target = 0;
    public static double power = 0;
    public boolean devBool = false;

    public static int[] POSITIONS = {LOW_BASKET, HIGH_BASKET, LOW_CHAMBER_SET, HIGH_CHAMBER_SET};

    // PID controller initialization
    private final PIDFController controller = new PIDFController(p, i, d,f);

    public OuttakeSlides(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {

        slideR = new MotorEx(hwMap, "rightSlide", Motor.GoBILDA.RPM_312);
        slideL = new MotorEx(hwMap, "leftSlide", Motor.GoBILDA.RPM_312);
        voltage = hwMap.get(VoltageSensor.class, "Control Hub");

        slideL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        reset();

        controller.setTolerance(5); // set tolerance for PID controller
    }

    public void downUntil() {
        setTarget(-9999);
        time.start();
        while (voltage.getVoltage() > 11 && !time.done()) { //12V is the minimum required to work fully
            update();
        }
        reset();
        restPos();
    }

    public void setTarget(double target) {
        OuttakeSlides.target = target;
    }

    public void goToPos(int pos) {
        setTarget(POSITIONS[pos]);
    }

    public void restPos() {
        setTarget(REST_POS);
    }

    public void intakePos(){
        setTarget(INTAKE_POS);
    }

    public void upABit() {
        setTarget(target + ABIT);
    }

    public void downABit() {
        setTarget(target - ABIT);
    }

    public void primeAscent(){
        setTarget(LEVEL_1_ASCENT);
    }

    public void ascent(){
        setTarget(HANG);
    }

    public void reset(){
        slideL.resetEncoder();
        slideR.resetEncoder();
    }

    public boolean isDone(){
        return controller.atSetPoint();
    }

    public void lock(){
        if (target == HIGH_CHAMBER_SET || target == LOW_CHAMBER_SET) {
            setTarget(target - CHAMBER_SCORED);
        }
    }

    public void setSlidePower(double slidePower){
        slideR.set(slidePower);
        slideL.set(-slidePower);
    }

    public double getError(){
        return controller.getPositionError();
    }

    public void update() {
        // Check values for the PID controller and update the power
        controller.setSetPoint(target);
        power = controller.calculate(slideR.getCurrentPosition());
        slideR.set(power);
        slideL.set(-power);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Target= ", target);
        telemetry.addData("Pos1= ", slideR.getCurrentPosition());
        telemetry.addData("is thingy work?", devBool);
        telemetry.addData("Current voltage: ", voltage.getVoltage());
    }

    @Override
    public void loop(FoozPad gamepad) {
        update();
        devBool = GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.RIGHT_BUMPER);
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.LOW_BASKET)) {
            goToPos(0);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.HIGH_BASKET)) {
            goToPos(1);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.LOW_SPECIMEN)) {
            goToPos(2);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.HIGH_SPECIMEN)) {
            goToPos(3);
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.LOCK_SPECIMEN)) {
            lock();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.GRAB_SPECIMEN)) {
            restPos();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.RIGHT_BUMPER)) {
            downUntil();
        }
    }
}