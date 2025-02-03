package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Config
@TeleOp
public class SlidesTuning extends OpMode {

    PIDController controller;
    public static double
            p = 0.02,
            i = 0,
            d = 0.0005,
            f = 0;
    MotorEx slideR, slideL;

    public static double target = 0;

    @Override
    public void init() {
        slideR = new MotorEx(hardwareMap, "rightSlide", Motor.GoBILDA.RPM_435);
        slideL = new MotorEx(hardwareMap, "leftSlide", Motor.GoBILDA.RPM_435);
        slideL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(p, i, d);

        controller.setTolerance(2); // set tolerance for PID controller

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideR.resetEncoder();
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        controller.setSetPoint(target);
        double power = controller.calculate(slideR.getCurrentPosition());
        slideR.set(power);
        slideL.set(-power);
        telemetry.addData("pos", slideR.getCurrentPosition());
        telemetry.addData("target", target);
    }
}
