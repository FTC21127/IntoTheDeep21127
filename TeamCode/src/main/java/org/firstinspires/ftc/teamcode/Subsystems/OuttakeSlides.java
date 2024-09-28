package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;


@Config
public class OuttakeSlides extends Mechanism {

    MotorEx[] slide = new MotorEx[2];

    MotorGroup slides;

    private double p, i, d, f;

    final double TICKSPERINCH = ((1+(46/11.0)) * 28) * (1.5*Math.PI); // Just for temp use for testing

    //Just random values right now, will tune later.
    public static int REST_POS = 0;
    public static int LOW_CHAMBER_SET = 155;
    public static int LOW_CHAMBER_SCORED = 145;
    public static int HIGH_CHAMBER_SET = 220;
    public static int HIGH_CHAMBER_SCORED = 210;
    public static int LOW_BASKET = 250;
    public static int HIGH_BASKET = 450;
    public static int LOW_RUNG = 250;
    public static int HIGH_RUNG = 350;
    public static int ABIT = 200;

    public double target = 0;
    public double power = 0;

    private final PIDController controller = new PIDController(p, i, d);

    public OuttakeSlides(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        slide[0] = new MotorEx(hwMap, "leftSlide", Motor.GoBILDA.RPM_1150);
        slide[1] = new MotorEx(hwMap, "rightSlide",Motor.GoBILDA.RPM_1150);
        slide[1].setInverted(true);

        slides = new MotorGroup(slide[0],slide[1]);
        slides.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }
}
