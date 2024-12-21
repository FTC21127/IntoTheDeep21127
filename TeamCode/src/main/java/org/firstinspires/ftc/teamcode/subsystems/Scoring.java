package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.auton.utils.Colors;


public class Scoring extends Mechanism {

    private Drivetrain drive = new Drivetrain(opMode);
    private OuttakeSlides slides = new OuttakeSlides(opMode);
    private Deposit deposit = new Deposit(opMode);
    private Intake intake;

    private int slidesPos = 0;
    private State state = State.INTAKE;
    private Colors color;
    private boolean isBasket = false;
    private boolean intakeGrabbed = false;
    private boolean intakeOut = false;
    private boolean isClimb = false;
    private int LB, HB, LC, HC;

    private enum State {
        INTAKE,
        TRANSFER,
        SCORING,
        CLIMB
    }

    // Slide Commands

    // Deposit Commands

    // Intake Commands


    // Transfer Slide Command Sequences

    // Non-transfer Slide Command Sequences

    // Intake Command Sequences

    // Ascend Command Sequences



    public Scoring(OpMode opMode, Colors color) {
        this.opMode = opMode;
        this.color = color;
    }

    @Override
    public void init(HardwareMap hwMap) {
        intake = new Intake(opMode,color);
        drive.init(hwMap);
        slides.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);
    }

    @Override
    public void loop(FoozPad gamepad1, FoozPad gamepad2) {

    }
}
