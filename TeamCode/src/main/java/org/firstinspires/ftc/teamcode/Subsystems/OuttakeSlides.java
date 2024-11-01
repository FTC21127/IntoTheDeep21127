package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.teleop.Controls;

/**
 * Done
 */
@Config   // @Config here is just gonna be used for easy tuning via FTC Dashboard
public class OuttakeSlides extends Mechanism {

    // This is the outtake slides class.
    // Includes a PID controller, set positions, and manual controls

    MotorEx slideR, slideL;

    // PID controller coefficients
    private double p = 0.015, i = 0, d = 0.0001, f = 0.01;

    // Positions for slides
    //Just random values right now, will tune later.
    public static int REST_POS = 90;
    public static int INTAKE_POS = 15;
    public static int LOW_BASKET = 500; //0
    public static int HIGH_BASKET = 1000; //1
    public static int LOW_CHAMBER_SET = 150; //2
    public static int HIGH_CHAMBER_SET = 420; //3
    public static int CHAMBER_SCORED = 110;
    public static int ABIT = 70;

    public static double target = 0;
    public static double power = 0;
    public static double power1 = 0;
    public static double minPower = -0.3;

    public static int[] POSITIONS = {LOW_BASKET, HIGH_BASKET, LOW_CHAMBER_SET, HIGH_CHAMBER_SET};

    // PID controller initialization
    private final PIDFController controller = new PIDFController(p, i, d,f);
    private final PIDFController controller1 = new PIDFController(p, i, d,f);

    public OuttakeSlides(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {

        // HardwareMap and init for other stuff
        slideL= new MotorEx(hwMap, "leftSlide", Motor.GoBILDA.RPM_1150);
        slideR = new MotorEx(hwMap, "rightSlide",Motor.GoBILDA.RPM_1150);
        slideL.setInverted(true);

        slideL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideL.resetEncoder();
        slideR.resetEncoder();

        controller.setTolerance(5); // set tolerance for PID controller
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

    public void reset(){
        slideL.resetEncoder();
        slideR.resetEncoder();
    }

    public void lock(){
        if (target == HIGH_CHAMBER_SET || target == LOW_CHAMBER_SET) {
            setTarget(target - CHAMBER_SCORED);
        }
    }

    public void update() {
        // Check values for the PID controller and update the power
        controller.setSetPoint(target);
        controller1.setSetPoint(target);
        power = controller.calculate(slideR.getCurrentPosition());
        power1 = controller1.calculate(slideL.getCurrentPosition());
        // set a max velocity for the motors
        // if (power<minPower) power = minPower;
        // if (power1>-minPower) power = minPower;
        slideR.set(power);
        slideL.set(-power1);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Target= ", target);
    }

    @Override
    public void loop(FoozPad gamepad) {
        update();
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
            intakePos();
        }
    }
}