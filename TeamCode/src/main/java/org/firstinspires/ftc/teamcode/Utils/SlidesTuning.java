package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
@TeleOp
public class SlidesTuning extends OpMode {

    PIDController controller;
    public static double
            p = 0,
            i = 0,
            d = 0,
            f = 0;
    public static double TICKSPERDEGREE = (1+(46/11.0)) * 28/360;
    public static int target;

    private MotorGroup motor;
    private Motor motor1, motor2;


    @Override
    public void init() {
        motor1 = new Motor(hardwareMap, "motor1");
        motor2 = new Motor(hardwareMap, "motor2");
        motor2.setInverted(true);
        motor = new MotorGroup(motor1,motor2);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos1 = motor.getCurrentPosition();
        double pid = controller.calculate(armPos1, target);
        double power = pid + f;
        motor.set(power);
        telemetry.addData("pos", motor.getCurrentPosition());
        telemetry.addData("target", target);

    }
}
