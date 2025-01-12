package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FoozFunctions.SimpleMotionProfile;

public class MotionProfileExample extends OpMode {
    // Declare pivot motor
    DcMotor pivot;
    // Create motor controller
    SimpleMotionProfile pivotController = new SimpleMotionProfile(0.01, 0.6, 3);

    @Override
    public void init() {
        // hardware map stuff
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        // input the current position into the controller
        double power = pivotController.calculate(pivot.getCurrentPosition());
        // You can also do:  pivotController.calculate(pivot.getCurrentPosition(), target position);
        // set pivot power
        pivot.setPower(power);

        if (gamepad1.a){
            // If gamepad 1 presses "a", set the target to 1000 (ticks)
            pivotController.setTarget(1000);
        } else if (gamepad1.b) {
            // If gamepad 1 presses "b", set the target to 0 (ticks)
            pivotController.setTarget(0);
        }
    }
}
