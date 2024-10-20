package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Deposit;


@Config
@TeleOp
public class SlidesTuning extends OpMode {

    PIDController controller;
    public static double
            p = 0.015,
            i = 0,
            d = 0.0001,
            f = 0;
    public static double TICKSPERDEGREE = (1+(46/11.0)) * 28/360;
    public static int target;

    private Motor motor1;
    private Motor.Encoder encoder;


    @Override
    public void init() {
        motor1 = new Motor(hardwareMap, "rightSlide");
        motor1.setInverted(true);
        encoder = motor1.encoder;
        encoder.reset();
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller.setTolerance(5);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        controller.setSetPoint(encoder.getPosition());
        double pid = controller.calculate(target);
        double power = pid - f;
        motor1.set(power);
        telemetry.addData("pos", motor1.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("pid value from controller", pid);
    }
}
