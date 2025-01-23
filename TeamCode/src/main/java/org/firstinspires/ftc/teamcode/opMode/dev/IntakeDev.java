package org.firstinspires.ftc.teamcode.opMode.dev;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;

@TeleOp(name = "Claw Dev", group = "Dev")
public class IntakeDev extends OpMode {
    Intake intake = new Intake(this, Intake.COLOR.RED);
    FoozPad foozPad;

    Command intakeSet = () -> {intake.barIntermediateDown();intake.neutralClaw();};
    Command intakeDown = () -> intake.barPickUp();
    Command intakeGrab = () -> intake.closeClaw();
    Command intakeOpen = () -> intake.openClaw();
    Command intakeTransfer = () -> intake.barTransfer();
    Command intakeNeutral = () -> intake.barNeutral();

    CommandSequence grabSample = new CommandSequence()
            .addCommand(intakeOpen)
            .addWaitCommand(.15)
            .addCommand(intakeDown)
            .addWaitCommand(.1)
            .addCommand(intakeGrab)
            .addWaitCommand(.2)
            .addCommand(intakeTransfer)
            .addWaitCommand(.8)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(intakeNeutral)
            .addCommand(intake::neutralClaw)
            .build();

    boolean isIntake = false;

    @Override
    public void init() {
        intake.init(hardwareMap);
        foozPad = new FoozPad(gamepad1);
    }

    @Override
    public void loop() {
        foozPad.update();
        intake.loop(foozPad);
//        if (GamepadStatic.isButtonPressed(foozPad.gamepad, GamepadStatic.Input.LEFT_BUMPER)){
//            if (!isIntake) intakeSet.run();
//            isIntake = true;
//        } else if (isIntake){
//            grabSample.trigger();
//            isIntake = false;
//        }
    }
}
