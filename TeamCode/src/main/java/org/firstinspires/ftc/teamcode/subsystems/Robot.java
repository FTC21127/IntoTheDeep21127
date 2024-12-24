package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.auton.utils.Colors;

public class Robot extends Mechanism {
    Scoring robot;
    Colors alliance;

    public Robot(OpMode opMode, Colors alliance) {
        this.alliance = alliance;
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        robot = new Scoring(opMode,alliance);
        robot.init(hwMap);
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {
        robot.loop(gamepad1, gamepad2);
    }

    public Scoring.State getState(){
        return robot.getState();
    }
}
