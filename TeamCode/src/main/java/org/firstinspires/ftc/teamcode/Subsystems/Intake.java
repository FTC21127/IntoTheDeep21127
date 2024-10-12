package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;

public class Intake extends Mechanism {

    public static int YELLOW = 10;
    public static int PURPLE = 10;
    public static int GREEN = 10;
    public static int WHITE = 10;

    public Intake(OpMode opMode1) {
        this.opMode = opMode1;
    }

    @Override
    public void init(HardwareMap hwMap) {

    }

    @Override
    public void loop(Gamepad gamepad) {
        super.loop(gamepad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry(telemetry);
    }


    private class IntakeSensor extends Mechanism {

        private ColorRangeSensor sensor;
        private String name;

        private double far;

        public IntakeSensor(OpMode opMode, String name, double far) {
            this.opMode = opMode;
            this.name = name;
            this.far = far;
        }

        public void init(HardwareMap hwMap) {
            sensor = hwMap.get(ColorRangeSensor.class, name);

            sensor.enableLed(false);
        }

        public boolean isSample() {
            return sensor.getDistance(DistanceUnit.MM) < far;
        }

        public boolean isSampleColor() {
            int blue = sensor.blue();
            int red = sensor.red();
            int green = sensor.green();
            int white = (blue + red + green) / 3;
            int yellow = (red + green) / 2;
            int purple = (red + blue) / 2;
            boolean isSample = white > WHITE || yellow > YELLOW || purple > PURPLE || green > GREEN;
            Telemetry t = FtcDashboard.getInstance().getTelemetry();
            t.addData(name + " isSample", isSample);
            t.update();
            return isSample;
        }

        @Override
        public void telemetry(Telemetry telemetry) {
            telemetry.addData(name + " dist", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
