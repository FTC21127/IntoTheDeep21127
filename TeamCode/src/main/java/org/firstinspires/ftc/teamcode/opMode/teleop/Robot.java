package org.firstinspires.ftc.teamcode.opMode.teleop;

import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadColors.SARAH_INTAKE;
import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadColors.SARAH_OUTTAKE;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.Controls;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadColors;

import java.util.concurrent.TimeUnit;

@TeleOp
public class Robot extends OpMode {
    Scoring bot = new Scoring(this, Intake.COLOR.RED);
    FoozPad gp1, gp2;
    Scoring.State previousState = Scoring.State.SCORING;
    Timing.Timer time = new Timing.Timer(2000, TimeUnit.MILLISECONDS);

    @Override
    public void init() {
        bot.init(hardwareMap);
        gp1 = new FoozPad(gamepad1);
        gp2 = new FoozPad(gamepad2);
        gp2.assignedPad.runLedEffect(SARAH_INTAKE.colorPattern);
        gp1.assignedPad.runLedEffect(FoozPadColors.NURAZ_DEFAULT.colorPattern);
    }

    @Override
    public void start() {
        bot.initM();
    }

    @Override
    public void loop() {
        bot.loop(gp1, gp2);
        gp1.update();
        gp2.update();
        if (!bot.state.equals(previousState)) {
            if (bot.state.equals(Scoring.State.INTAKE)) {
                gp2.assignedPad.runLedEffect(SARAH_INTAKE.colorPattern);
            } else if (bot.state.equals(Scoring.State.SCORING)) {
                gp2.assignedPad.runLedEffect(SARAH_OUTTAKE.colorPattern);
                gp1.assignedPad.runLedEffect(FoozPadColors.NURAZ_DEFAULT.colorPattern);
            } else {
                gp2.assignedPad.setLedColor(1,1,1, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }
        previousState = bot.getState();
    }
}
