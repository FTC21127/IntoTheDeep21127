package org.firstinspires.ftc.teamcode.opMode.dev;

import static org.firstinspires.ftc.teamcode.opMode.dev.ClawTest.COLOR.BLUE;
import static org.firstinspires.ftc.teamcode.opMode.dev.ClawTest.COLOR.NONE;
import static org.firstinspires.ftc.teamcode.opMode.dev.ClawTest.COLOR.RED;
import static org.firstinspires.ftc.teamcode.opMode.dev.ClawTest.COLOR.YELLOW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Claw Dev", group = "Dev")
public class ClawTest extends OpMode {
    Servo claw;
    SensorColor color;
    public static double pos = 0;

    enum COLOR{
        BLUE(1),
        RED(2),
        YELLOW(3),
        NONE(0);
        int i;
        COLOR(int j) {
            i = j;
        }
    }
    COLOR sampleColor = NONE;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = hardwareMap.get(Servo.class, "outtakeClaw");
        color = new SensorColor(hardwareMap, "color");
        claw.setPosition(pos);
    }

    @Override
    public void loop() {
        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Green: ", color.green());
        telemetry.addData("Claw: ", claw.getPosition());
        if (gamepad1.right_bumper){
            openClaw();
        }
        if (gamepad1.left_bumper && sampleColor.i != 0){
            closeClaw();
        }
        if (color.red() > (color.blue() + color.green()) * .75){
            sampleColor = RED;
        } else if (color.blue() > (color.red() + color.green()) * .75){
            sampleColor = BLUE;
        } else if (color.green() > 1000 && color.red() > 1000 && !(color.blue() > 1000)){
            sampleColor = YELLOW;
        } else {
            sampleColor = NONE;
        }

    }

    public void closeClaw(){
        claw.setPosition(0);
    }

    public void openClaw(){
        claw.setPosition(1);
    }
}
