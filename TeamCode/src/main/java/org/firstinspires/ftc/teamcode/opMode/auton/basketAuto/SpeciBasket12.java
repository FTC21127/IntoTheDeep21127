package org.firstinspires.ftc.teamcode.opMode.auton.basketAuto;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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


@Autonomous(name = "1 + 1' high", group = "!basket", preselectTeleOp = "Robot")
public class SpeciBasket12 extends OpMode {

    Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private Follower follower;

    private Path specimen, sample1, basket1, sample2, basket2, park;

    private Intake intake = new Intake(this);
    private OuttakeSlides slides = new OuttakeSlides(this);
    private Deposit deposit = new Deposit(this);

    private boolean busy = false, busy2 = false;
    private int SLIDE_ERROR = 10;

    private Command specimenCommand = () -> follower.followPath(specimen);
    private Command sample1Command = () -> follower.followPath(sample1, true);
    private Command basketCommand = () -> follower.followPath(basket1, true);
    private Command sample2Command = () -> follower.followPath(sample2, true);
    private Command basket2Command = () -> follower.followPath(basket2, true);
    private final Command busyTrue = () -> busy = true;
    private final Command busyFalse = () -> busy = false;
    private final Command highSlideError = () -> SLIDE_ERROR = 100;

    // Slide Commands
    private final Command slidesIntake = () -> slides.intakePos();
    private final Command slideRest = () -> slides.setTarget(OuttakeSlides.REST_POS+200);
    private final Command highSpecimen = () -> slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET-150);
    private final Command highBasket = () -> {slides.setTarget(OuttakeSlides.HIGH_BASKET); deposit.basketPos();};
    private final Command lockSpecimen = () -> slides.lock();
    private final Command atPos = () -> slides.isDone();
    // Deposit Commands
    private final Command depositPos = () -> deposit.depositPos();
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command basketPos = () -> deposit.basketPos();
    private final Command specimenPos = () -> deposit.specimenSetPos();
    private final Command specimenScorePos = () -> deposit.specimenScorePos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    // Intake Commands
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command dropV4b = () -> intake.barDown();
    private final Command neutralV4b = () -> intake.barNeutral();
    private final Command transferV4b = () -> intake.barTransfer();

    private CommandSequence initSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addCommand(grabTransfer)
            .addCommand(neutralV4b)
            .addCommand(slideRest)
            .build();

    private CommandSequence move1 = new CommandSequence()
            .addCommand(highSpecimen)
            .addCommand(outtakeGrab)
            .addCommand(specimenPos)
            .addCommand(specimenCommand)
            .build();
    private CommandSequence scoreSpecimen = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(0.1)
            .addCommand(specimenScorePos)
            .addWaitCommand(.4)
            .addCommand(lockSpecimen)
            .addWaitCommand(0.3)
            .addCommand(outtakeRelease)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2 = new CommandSequence()
            .addCommand(sample1Command)
            .addWaitCommand(.3)
            .addCommand(outtakeGrab)
            .addWaitCommand(.3)
            .addCommand(slideRest)
            .addWaitCommand(.1)
            .addCommand(dropV4b)
            .addWaitCommand(.5)
            .addCommand(intakeOpen)
            .build();
    private CommandSequence transferSequence = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeGrab)
            .addWaitCommand(0.5)
            .addCommand(transferV4b)
            .addCommand(outtakeGrab)
            .addWaitCommand(1.2)
            .addCommand(intakeOpen)
            .addWaitCommand(.5)
            .addCommand(neutralV4b)
            .addWaitCommand(.1)
            .addCommand(grabTransfer)
            .addCommand(intakeGrab)
            .addWaitCommand(.5)
            .addCommand(outtakeRelease)
            .addCommand(slidesIntake)
            .addWaitCommand(.6)
            .addCommand(outtakeGrab)
            .addCommand(highSlideError)
            .addCommand(highSlideError)
            .addWaitCommand(.6)
            .addCommand(highBasket)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move3 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(basketCommand)
            .addCommand(basketPos)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(.5)
            .addCommand(outtakeRelease)
            .addCommand(outtakeRelease)
            .addWaitCommand(.5)
            .addCommand(grabTransfer)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move4 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample2Command)
            .addCommand(dropV4b)
            .addWaitCommand(.5)
            .addCommand(depositPos)
            .addCommand(slideRest)
            .addCommand(intakeOpen)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move5 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(basket2Command)
            .addCommand(basketPos)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence holdEnd = new CommandSequence()
            .addCommand(slideRest)
            .addCommand(busyTrue)
            .addCommand(busyTrue)
            .build();

    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(move1)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(move2)
            .addCommandSequence(transferSequence)
            .addCommandSequence(move3)
            .addCommandSequence(scoreBasket)
            .addCommandSequence(move4)
            .addCommandSequence(transferSequence)
            .addCommandSequence(move5)
            .addCommandSequence(scoreBasket)
            .addCommandSequence(holdEnd)
            .build();

    @Override
    public void init() {

        intake.init(hardwareMap);
        slides.init(hardwareMap);
        deposit.init(hardwareMap);
        follower = new Follower(hardwareMap);

        follower.setPose(new Pose(8.836, 80, Math.toRadians(180)));

        specimen = new Path(new BezierLine(
                new Point(8.836, 80, Point.CARTESIAN),
                new Point(36.500, 82.000, Point.CARTESIAN)));
        specimen.setConstantHeadingInterpolation(Math.toRadians(180));
        sample1 = new Path(new BezierCurve(
                new Point(36.500, 82.000, Point.CARTESIAN),
                new Point(29.138, 91.927, Point.CARTESIAN),
                new Point(46.5, 120, Point.CARTESIAN)));
        sample1.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));
        basket1 = new Path(new BezierLine(
                new Point(46.5, 120, Point.CARTESIAN),
                new Point(30, 122, Point.CARTESIAN)));
        basket1.setConstantHeadingInterpolation(Math.toRadians(-45));
        sample2 = new Path(new BezierLine(
                new Point(30, 122, Point.CARTESIAN),
                new Point(46.5, 130, Point.CARTESIAN)));
        sample2.setConstantHeadingInterpolation(Math.toRadians(0));
        basket2 = new Path(new BezierLine(
                new Point(46.5, 130, Point.CARTESIAN),
                new Point(30, 122, Point.CARTESIAN)));
        basket2.setConstantHeadingInterpolation(Math.toRadians(-45));

        initSequence.trigger();
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
//        if (commandMachine.getCurrentCommandIndex() < 6) {
            commandMachine.run(busy2 || busy);
//        }
        follower.update();
        slides.update();
        busy2 = follower.isBusy() || Math.abs(slides.getError())>SLIDE_ERROR;
        telemetry.addData("is busy", busy);
        telemetry.addData("follower busy? ", busy2);
        telemetry.addData("current command: ", commandMachine.getCurrentCommandIndex());
        telemetry.addData("current error: ", slides.getError());
        telemetry.addData("current pos: ", follower.getPose());
    }
}


//public class GeneratedPath {
//
//  public GeneratedPath() {
//    PathBuilder builder = new PathBuilder();
//
//    builder
//      .addPath(
//        // Line 1
//        new BezierLine(
//          new Point(7.708, 104.898, Point.CARTESIAN),
//          new Point(13.347, 125.953, Point.CARTESIAN)
//        )
//      )
//      .setConstantHeadingInterpolation(Math.toRadians(-45))
//      .addPath(
//        // Line 2
//        new BezierLine(
//          new Point(13.347, 125.953, Point.CARTESIAN),
//          new Point(37.598, 121.441, Point.CARTESIAN)
//        )
//      )
//      .setConstantHeadingInterpolation(Math.toRadians(0))
//      .addPath(
//        // Line 3
//        new BezierLine(
//          new Point(37.598, 121.441, Point.CARTESIAN),
//          new Point(15.979, 131.593, Point.CARTESIAN)
//        )
//      )
//      .setConstantHeadingInterpolation(Math.toRadians(-40))
//      .addPath(
//        // Line 4
//        new BezierLine(
//          new Point(15.979, 131.593, Point.CARTESIAN),
//          new Point(37.034, 131.217, Point.CARTESIAN)
//        )
//      )
//      .setConstantHeadingInterpolation(Math.toRadians(0))
//      .addPath(
//        // Line 5
//        new BezierLine(
//          new Point(37.034, 131.217, Point.CARTESIAN),
//          new Point(14.475, 132.533, Point.CARTESIAN)
//        )
//      )
//      .setConstantHeadingInterpolation(Math.toRadians(-45))
//      .addPath(
//        // Line 6
//        new BezierCurve(
//          new Point(14.475, 132.533, Point.CARTESIAN),
//          new Point(62.789, 114.110, Point.CARTESIAN),
//          new Point(63.540, 95.687, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));
//  }
//}