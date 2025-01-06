package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public Rev2mDistanceSensor resetSensor;

    //Use voltage sensor
    public VoltageSensor voltage;
    Timing.Timer time = new Timing.Timer(2000, TimeUnit.MILLISECONDS);

    // PID controller coefficients
    private final double p = 0.0175, i = 0, d = 0.0005, f = 0;

    // Positions for slides
    public static int REST_POS = 95;
    public static int INTAKE_POS = 10;
    public static int LOW_BASKET = 2000; //0
    public static int HIGH_BASKET = 3800; //1
    public static int LOW_CHAMBER_SET = 400; //2
    public static int HIGH_CHAMBER_SET = 1450; //3
    public static int CHAMBER_SCORED = 500;
    public static int LEVEL_1_ASCENT = 2000;
    public static int HANG = -800;
    public static int ABIT = 100;

    public static double target = 0;
    public static double power = 0;
    public boolean devBool = false;
    public boolean reset = true;

    public static int[] POSITIONS = {LOW_BASKET, HIGH_BASKET, LOW_CHAMBER_SET, HIGH_CHAMBER_SET};

    // PID controller initialization
    private final PIDFController controller = new PIDFController(p, i, d,f);

    public OuttakeSlides(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        resetSensor = hwMap.get(Rev2mDistanceSensor.class, "slideReset");
        slideR = new MotorEx(hwMap, "rightSlide", Motor.GoBILDA.RPM_312);
        slideL = new MotorEx(hwMap, "leftSlide", Motor.GoBILDA.RPM_312);
        voltage = hwMap.get(VoltageSensor.class, "Control Hub");

        slideL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller.setTolerance(5); // set tolerance for PID controller
    }

    public void downUntil() {
        reset = false;
        resetSensor.getDistance(DistanceUnit.CM);
        setSlidePower(-.3);
        time.start();
        while (voltage.getVoltage() > 10.5 && !time.done() && resetSensor.getDistance(DistanceUnit.CM) > 7.5) { //12V is the minimum required to work fully
        }
        reset();
        intakePos();
        reset = true;
    }

    public void downUntil(OpMode mode, FoozPad gp, HardwareMap hwMap) {
        resetSensor.getDistance(DistanceUnit.CM);
        Drivetrain dt = new Drivetrain(mode);
        dt.init(hwMap);
        gp.update();
        setSlidePower(-.4);
        time.start();
        while (voltage.getVoltage() > 10.5 && !time.done() && resetSensor.getDistance(DistanceUnit.CM) > 7.4) { //12V is the minimum required to work fully
            gp.update();
            dt.loop(gp);
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
        telemetry.addData("Distance (CM): ", resetSensor.getDistance(DistanceUnit.CM));
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
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.RELEASE)) {
            lock();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, Controls.GRAB_SPECIMEN)) {
            restPos();
        } else if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.RIGHT_BUMPER)) {
            downUntil();
        }
    }
}