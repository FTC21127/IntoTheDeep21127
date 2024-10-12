package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.teleop.Controls;

/**Done (probably)*/
@Config   // @Config here is just gonna be used for easy tuning via FTC Dashboard
public class OuttakeSlides extends Mechanism {

    // This is the outtake slides class.
    // Includes a PID controller, set positions, and manual controls

    MotorEx slideR, slideL;

    MotorGroup slides;

    // PID controller coefficients
    private double p = 0.015, i = 0, d = 0.0001, f = 0;

    // Positions for slides
    //Just random values right now, will tune later.
    public static int REST_POS = 20;
    public static int INTAKE_POS = 0;
    public static int LOW_BASKET = 250; //1
    public static int HIGH_BASKET = 450; //2
    public static int LOW_CHAMBER_SET = 155; //3
    public static int HIGH_CHAMBER_SET = 220; //4
    public static int HIGH_CHAMBER_SCORED = 210;
    public static int LOW_CHAMBER_SCORED = 145;
    public static int ABIT = 70;

    public static int[] POSITIONS = {LOW_BASKET, HIGH_BASKET, LOW_CHAMBER_SET, HIGH_CHAMBER_SET};

    public static double target = 0;
    public static double power = 0;
    public static double minPower = -0.3;

    // PID controller initialization
    private final PIDController controller = new PIDController(p, i, d);

    public OuttakeSlides(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {

        // HardwareMap and init for other stuff
        slideL= new MotorEx(hwMap, "leftSlide", Motor.GoBILDA.RPM_1150);
        slideR = new MotorEx(hwMap, "rightSlide",Motor.GoBILDA.RPM_1150);
        slideL.setInverted(true);

        slides = new MotorGroup(slideL,slideR);
        slides.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        controller.setTolerance(5); // set tolerance for PID controller
    }

    public void setTarget(double target) {
        OuttakeSlides.target = target;
    }

    public double getPosition() {
        return slides.getCurrentPosition();
    }

    public void goToPos(int pos) {
        setTarget(POSITIONS[pos]);
    }

    public void restPos() {
        setTarget(REST_POS);
    }

    public void upABit() {
        setTarget(target + ABIT);
    }

    public void downABit() {
        setTarget(target - ABIT);
    }

    public void update() {
        // Check values for the PID controller and update the power
        controller.setSetPoint(target);
        power = controller.calculate(getPosition());
        if (power < minPower) {
            power = minPower;
        }
        slides.set(power);
    }

    @Override
    public void loop(Gamepad gamepad) {
        update();
        if (GamepadStatic.isButtonPressed(gamepad, Controls.LOW_BASKET)) {
            goToPos(0);
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.HIGH_BASKET)) {
            goToPos(1);
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.LOW_SPECIMEN)) {
            goToPos(2);
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.HIGH_SPECIMEN)) {
            goToPos(3);
        }
    }
}