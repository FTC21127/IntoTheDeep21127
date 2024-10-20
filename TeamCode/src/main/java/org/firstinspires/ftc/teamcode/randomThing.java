package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class randomThing extends OpMode {
    DcMotor m1, m2 , m3;
    DcMotor[] motors;

    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotor.class, "frontLeft");
        m2 = hardwareMap.get(DcMotor.class, "frontRight");
        m3 = hardwareMap.get(DcMotor.class, "backLeft");
        motors = new DcMotor[]{m1, m2, m3};
        for (DcMotor m: motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(1);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    @Override
    public void loop() {
        telemetry.addData("Left: ", motors[0].getCurrentPosition());
        telemetry.addData("Strafe: ", motors[1].getCurrentPosition());
        telemetry.addData("Right: ", motors[2].getCurrentPosition());
    }
}
