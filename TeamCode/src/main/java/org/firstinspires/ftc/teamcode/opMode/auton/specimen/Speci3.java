package org.firstinspires.ftc.teamcode.opMode.auton.specimen;

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


@Autonomous(name = "3 + 0'", group = "!specimen", preselectTeleOp = "Robot")
public class Speci3 extends OpMode {

    Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private Follower follower;

    private Path preload, alignSample, yeetSample, pickupSpeci2, scoreSpeci2, alignSpeci3, pickupSpeci3, scoreSpeci3, park;

    private final Intake intake = new Intake(this);
    private final OuttakeSlides slides = new OuttakeSlides(this);
    private final Deposit deposit = new Deposit(this);

    private boolean busy = false, busy2 = false;

    private final Command preloadCommand = () -> follower.followPath(preload);
    private final Command specimen2Command = () -> {follower.followPath(scoreSpeci2, false);follower.setMaxPower(1);};
    private final Command specimen3Command = () -> follower.followPath(scoreSpeci3, false);
    private final Command alignSampleCommand = () -> follower.followPath(alignSample, true);
    private final Command ejectCommand = () -> {follower.followPath(yeetSample, true);follower.setMaxPower(1);};
    private final Command pickupSpecimen2Command = () -> follower.followPath(pickupSpeci2, true);
    private final Command pickupSpecimen3Command = () -> follower.followPath(pickupSpeci3, true);
    private final Command alignSpecimen3Command = () -> follower.followPath(alignSpeci3, true);
    private final Command parkCommand = () -> follower.followPath(park, false);

    private final Command busyTrue = () -> busy = true;
    private final Command busyFalse = () -> busy = false;
    private final Command forceQuit = () -> {busy2 = false;busy = false;};

    // Slide Commands
    private final Command slidesIntake = slides::intakePos;
    private final Command slideRest = () -> slides.setTarget(OuttakeSlides.REST_POS + 200);
    private final Command slideSpecimen = () -> slides.setTarget(OuttakeSlides.REST_POS);
    private final Command highSpecimen = () -> slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET - 115);
    private final Command lockSpecimen = slides::lock;
    // Deposit Commands
    private final Command depositPos = deposit::depositPos;
    private final Command grabTransfer = deposit::transferPos;
    private final Command specimenPos = deposit::specimenSetPos;
    private final Command specimenScorePos = deposit::specimenScorePos;
    private final Command outtakeRelease = deposit::openClaw;
    private final Command outtakeGrab = deposit::closeClaw;
    private final Command clawShift = deposit::clawShift;
    // Intake Commands
    private final Command intakeGrab = intake::closeClaw;
    private final Command intakeOpen = intake::openClaw;
    private final Command autonV4b = intake::autonDown;
    private final Command pickUpV4b = intake::barPickUp;
    private final Command neutralV4b = intake::barNeutral;
    private final Command transferV4b = intake::barTransfer;

    private final CommandSequence initSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addCommand(grabTransfer)
            .addCommand(neutralV4b)
            .addCommand(slideRest)
            .build();
    private final CommandSequence specimen1 = new CommandSequence()
            .addCommand(highSpecimen)
            .addCommand(outtakeGrab)
            .addCommand(specimenPos)
            .addCommand(preloadCommand)
            .addWaitCommand(.2)
            .build();
    private final CommandSequence scoreSpecimen = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(0.1)
            .addCommand(specimenScorePos)
            .addCommand(specimenScorePos)
            .addWaitCommand(.3)
            .addCommand(lockSpecimen)
            .addWaitCommand(0.2)
            .addCommand(outtakeRelease)
            .addCommand(outtakeRelease)
            .addWaitCommand(.05)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence samplePickUp = new CommandSequence()
            .addCommand(alignSampleCommand)
            .addWaitCommand(.3)
            .addCommand(autonV4b)
            .addCommand(depositPos)
            .addWaitCommand(.4)
            .addCommand(slideSpecimen)
            .addCommand(intakeOpen)
            .build();
    private final CommandSequence transferSequence = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(slidesIntake)
            .addCommand(pickUpV4b)
            .addCommand(intakeGrab)
            .addWaitCommand(0.4)
            .addCommand(transferV4b)
            .addCommand(outtakeRelease)
            .addCommand(ejectCommand)
            .addWaitCommand(.9)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(neutralV4b)
            .addWaitCommand(.3)
            .addCommand(grabTransfer)
            .addCommand(intakeGrab)
            .addWaitCommand(.4)
            .addCommand(outtakeGrab)
            .addWaitCommand(.2)
            .addCommand(slideSpecimen)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence alignSpecimen2  = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(depositPos)
            .addWaitCommand(.3)
            .addCommand(pickupSpecimen2Command)
            .addCommand(outtakeRelease)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence speci2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.2)
            .addCommand(specimenPos)
            .addWaitCommand(.1)
            .addCommand(specimen2Command)
            .addCommand(highSpecimen)
            .addCommand(highSpecimen)
            .addCommand(specimenPos)
            .addCommand(specimenPos)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence alignSpecimen3  = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(alignSpecimen3Command)
            .addCommand(depositPos)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence pickUpSpecimen3  = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(pickupSpecimen3Command)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen) // just in case
            .addCommand(depositPos)
            .addCommand(outtakeRelease)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence speci3 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.2)
            .addCommand(specimenPos)
            .addWaitCommand(.1)
            .addCommand(specimen3Command)
            .addCommand(highSpecimen)
            .addCommand(highSpecimen)
            .addCommand(specimenPos)
            .addCommand(specimenPos)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addWaitCommand(2)
            .addCommand(forceQuit)
            .addCommand(busyFalse)
            .build();

    private final CommandSequence holdEnd = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(parkCommand)
            .addCommand(busyTrue)
            .addCommand(busyTrue)
            .build();

    private final AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(specimen1)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(samplePickUp)
            .addCommandSequence(transferSequence)
            .addCommandSequence(alignSpecimen2)
            .addCommandSequence(speci2)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(alignSpecimen3)
            .addCommandSequence(pickUpSpecimen3)
            .addCommandSequence(speci3)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(holdEnd)
            .build();

    @Override
    public void init() {
        intake.init(hardwareMap);
        slides.init(hardwareMap);
        deposit.init(hardwareMap);
        slides.reset();
        follower = new Follower(hardwareMap);

        follower.setPose(new Pose(8.1, 64, Math.toRadians(180)));

        preload = new Path(new BezierLine(
                new Point(8.100, 64.000, Point.CARTESIAN),
                new Point(37.00, 60.000, Point.CARTESIAN)));
        preload.setConstantHeadingInterpolation(Math.toRadians(180));

        alignSample = new Path(new BezierCurve(
                new Point(37.00, 60.000, Point.CARTESIAN),
                new Point(28.000, 58.000, Point.CARTESIAN),
                new Point(45.50, 25.00, Point.CARTESIAN)));
        alignSample.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-20));

        yeetSample = new Path(new BezierLine(
                new Point(45.50, 25.00, Point.CARTESIAN),
                new Point(40.000, 30.000, Point.CARTESIAN)));
        yeetSample.setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0));

        pickupSpeci2 = new Path(new BezierLine(
                new Point(40.000, 30.000, Point.CARTESIAN),
                new Point(26.000, 30.000, Point.CARTESIAN)));
        pickupSpeci2.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreSpeci2 = new Path(new BezierCurve(
                new Point(26.000, 30.000, Point.CARTESIAN),
                new Point(10.000, 40.000, Point.CARTESIAN),
                new Point(20.000, 75.000, Point.CARTESIAN),
                new Point(38.000, 66.000, Point.CARTESIAN)));
        scoreSpeci2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        alignSpeci3 = new Path(new BezierCurve(
                new Point(38.000, 66.000, Point.CARTESIAN),
                new Point(24.000, 50.000, Point.CARTESIAN),
                new Point(30.000, 29.000, Point.CARTESIAN)));
        alignSpeci3.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        pickupSpeci3 = new Path(new BezierLine(
                new Point(30.000, 29.000, Point.CARTESIAN),
                new Point(26.000, 29.000, Point.CARTESIAN)));
        pickupSpeci3.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreSpeci3 = new Path(new BezierCurve(
                new Point(26.000, 29.000, Point.CARTESIAN),
                new Point(10.000, 40.000, Point.CARTESIAN),
                new Point(20.000, 75.000, Point.CARTESIAN),
                new Point(38.000, 69.000, Point.CARTESIAN)));
        scoreSpeci3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        park = new Path(new BezierCurve(
                new Point(38.000, 69.000, Point.CARTESIAN),
                new Point(10.000, 35.000, Point.CARTESIAN),
                new Point(28.000, 15.000, Point.CARTESIAN)));
        park.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        initSequence.trigger();
    }

    @Override
    public void init_loop() {
        slides.update();
        if (gamepad1.cross) {
            deposit.closeClaw();
        }
    }

    @Override
    public void start() {
        commandMachine.run(busy);
        slides.update();
        follower.update();
        timer.start();
        while (!timer.done()) {
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
        int SLIDE_ERROR = 10;
        busy2 = follower.isBusy() || Math.abs(slides.getError()) > SLIDE_ERROR;
        telemetry.addData("is busy", busy);
        telemetry.addData("follower busy? ", busy2);
        telemetry.addData("current command: ", commandMachine.getCurrentCommandIndex());
        telemetry.addData("current error: ", slides.getError());
        telemetry.addData("current pos: ", follower.getPose().getX());
        telemetry.addData("Current Path: ", follower.getCurrentPath());
    }
}