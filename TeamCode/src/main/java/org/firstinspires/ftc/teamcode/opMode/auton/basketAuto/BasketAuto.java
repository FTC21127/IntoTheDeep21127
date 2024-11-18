package org.firstinspires.ftc.teamcode.opMode.auton.basketAuto;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.fissionlib.command.AutoCommandMachine;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;


@Autonomous
public class BasketAuto extends OpMode {
    private Telemetry telemetryA;
    Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private Follower follower;

    private Path specimen, sample1, basket, park;

    private Intake intake = new Intake(this);
    private OuttakeSlides slides = new OuttakeSlides(this);
    private Deposit deposit = new Deposit(this);

    private boolean busy = false, busy2 = false;

    private Command specimenCommand = () -> follower.followPath(specimen);
    private Command sample1Command = () -> follower.followPath(sample1, true);
    private Command basketCommand = () -> follower.followPath(basket);
    private Command busyTrue = () -> busy = true;
    private Command busyFalse = () -> busy = false;

    // Slide Commands
    private final Command slidesIntake = () -> slides.intakePos();
    private final Command slideRest = () -> slides.restPos();
    private final Command highSpecimen = () -> slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET-40);
    private final Command lowBasket = () -> slides.setTarget(OuttakeSlides.LOW_BASKET);
    private final Command lockSpecimen = () -> slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET-240);
    // Deposit Commands
    private final Command depositPos = () -> deposit.depositPos();
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command basketPos = () -> deposit.basketPos();
    private final Command specimenPos = () -> deposit.specimenPos();
    private final Command specimenScorePos = () -> deposit.specimenScorePos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    // Intake Commands
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command dropV4b = () -> intake.barDown();
    private final Command neutralV4b = () -> intake.barNeutral();
    private final Command transferV4b = () -> intake.barTransfer();


    private CommandSequence move1 = new CommandSequence()
            .addCommand(highSpecimen)
            .addCommand(outtakeGrab)
            .addCommand(basketPos)
            .addCommand(specimenCommand)
            .build();
    private CommandSequence scoreSpecimen = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(0.1)
            .addCommand(specimenScorePos)
            .addWaitCommand(.7)
            .addCommand(lockSpecimen)
            .addCommand(outtakeRelease)
            .addWaitCommand(.5)
            .addCommand(grabTransfer)
            .addWaitCommand(1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2 = new CommandSequence()
            .addCommand(sample1Command)
            .addCommand(dropV4b)
            .addCommand(intakeOpen)
            .addCommand(busyTrue)
            .build();
    private CommandSequence transferSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addWaitCommand(.4)
            .addCommand(transferV4b)
            .addWaitCommand(.5)
            .addCommand(outtakeRelease)
            .addWaitCommand(.3)
            .addCommand(neutralV4b)
            .build();
    private CommandSequence move3 = new CommandSequence()
            .addCommand(basketCommand)
            .addCommand(slidesIntake)
            .addCommand(busyTrue)
            .build();

    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(move1)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(move2)
            .build();;

    @Override
    public void init() {
        intake.init(hardwareMap);
        slides.init(hardwareMap);
        deposit.init(hardwareMap);
        follower = new Follower(hardwareMap);

        follower.setPose(new Pose(8.836, 80.460, Math.toRadians(180)));

        specimen = new Path(new BezierLine(
                new Point(8.836, 80.460, Point.CARTESIAN),
                new Point(36.000, 78.000, Point.CARTESIAN)));
        specimen.setConstantHeadingInterpolation(Math.toRadians(180));
        sample1 = new Path(new BezierCurve(
                new Point(36.000, 78.000, Point.CARTESIAN),
                new Point(29.138, 91.927, Point.CARTESIAN),
                new Point(48.486, 118.569, Point.CARTESIAN)));
        sample1.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));
        basket = new Path(new BezierLine(
                new Point(48.486, 118.569, Point.CARTESIAN),
                new Point(32.5, 123, Point.CARTESIAN)));
        basket.setConstantHeadingInterpolation(Math.toRadians(-45));

        deposit.depositPos();
        intake.barNeutral();
        slides.setTarget(OuttakeSlides.INTAKE_POS);
    }

    @Override
    public void init_loop() {
        slides.update();
        if (gamepad1.cross){
            deposit.clawShift();
        }
    }

    @Override
    public void start() {
        commandMachine.run(busy);
        slides.update();
        follower.update();
        timer.start();
        while (!timer.done()){
            follower.update();
            slides.update();
        }
        busy2 = true;
    }

    @Override
    public void loop() {
        commandMachine.run(busy2 || busy);
        follower.update();
        slides.update();
        busy2 = follower.isBusy() || Math.abs(slides.getError())>10;
        telemetry.addData("is busy", busy);
        telemetry.addData("follower busy? ", busy2);
        telemetry.addData("current command: ", commandMachine.getCurrentCommandIndex());
        telemetry.addData("current error: ", slides.getError());
    }
}
