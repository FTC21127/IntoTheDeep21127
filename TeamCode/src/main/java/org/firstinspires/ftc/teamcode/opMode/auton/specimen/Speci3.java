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
    private final Command yeetSampleCommand = () -> follower.followPath(yeetSample, true);
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
    private final Command highSpecimen = () -> slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET);
    private final Command lockSpecimen = slides::lock;
    // Deposit Commands
    private final Command depositPos = deposit::depositPos;
    private final Command grabTransfer = deposit::transferPos;
    private final Command specimenPos = deposit::specimenSetPos;
    private final Command specimenScorePos = deposit::specimenScorePos;
    private final Command outtakeRelease = deposit::openClaw;
    private final Command outtakeGrab = deposit::closeClaw;
    private final Command clawShift = deposit::clawShift;
    private final Command initPos = deposit::initPos;
    // Intake Commands
    private final Command intakeGrab = intake::closeClaw;
    private final Command intakeOpen = intake::openClaw;
    private final Command autonV4b = intake::autonDown;
    private final Command pickUpV4b = intake::barPickUp;
    private final Command neutralV4b = intake::barNeutral;
    private final Command transferV4b = intake::barTransfer;

    private final CommandSequence initSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addCommand(initPos)
            .addCommand(neutralV4b)
            .addCommand(slideRest)
            .build();
    private final CommandSequence specimen1 = new CommandSequence()
            .addCommand(this::mSpecimenSet)
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
            .addCommand(busyTrue)
            .addCommand(alignSampleCommand)
            .addWaitCommand(.4)
            .addCommand(this::mSlideRest)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence push = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(yeetSampleCommand)
            .addWaitCommand(0.3)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence alignSpecimen2  = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(0.5)
            .addCommand(outtakeRelease)
            .addCommand(pickupSpecimen2Command)
            .addCommand(outtakeRelease)
            .addWaitCommand(0.4)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence speci2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.2)
            .addCommand(specimen2Command)
            .addCommand(this::mSpecimenSet)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence alignSpecimen3  = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(alignSpecimen3Command)
            .addWaitCommand(.4)
            .addCommand(this::mSlideRest)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence pickUpSpecimen3  = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(outtakeRelease)
            .addWaitCommand(.6)
            .addCommand(pickupSpecimen3Command)
            .addWaitCommand(.1)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence speci3 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.1)
            .addCommand(this::mSpecimenSet)
            .addWaitCommand(.1)
            .addCommand(specimen3Command)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(busyFalse)
            .build();

    private final CommandSequence holdEnd = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(parkCommand)
            .addWaitCommand(.4)
            .addCommand(this::mSlideRest)
            .addCommand(busyTrue)
            .build();

    private final AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(specimen1)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(samplePickUp)
            .addCommandSequence(push)
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
                new Point(37.00, 62.000, Point.CARTESIAN)));
        preload.setConstantHeadingInterpolation(Math.toRadians(180));

        alignSample = new Path(new BezierCurve(
                new Point(37.000, 62.000, Point.CARTESIAN),
                new Point(10.995, 35.394, Point.CARTESIAN),
                new Point(72, 26, Point.CARTESIAN),
                new Point(68.000, 22.000, Point.CARTESIAN)));
        alignSample.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        yeetSample = new Path(new BezierLine(
                new Point(68.000, 22.000, Point.CARTESIAN),
                new Point(34.000, 20.000, Point.CARTESIAN)));
        yeetSample.setConstantHeadingInterpolation(Math.toRadians(0));

        pickupSpeci2 = new Path(new BezierLine(
                new Point(34.000, 20.00, Point.CARTESIAN),
                new Point(25.500, 20.000, Point.CARTESIAN)));
        pickupSpeci2.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreSpeci2 = new Path(new BezierCurve(
                new Point(25.500, 20.000, Point.CARTESIAN),
                new Point(10.000, 40.000, Point.CARTESIAN),
                new Point(20.000, 75.000, Point.CARTESIAN),
                new Point(38.000, 64.000, Point.CARTESIAN)));
        scoreSpeci2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        alignSpeci3 = new Path(new BezierCurve(
                new Point(38.000, 64.000, Point.CARTESIAN),
                new Point(24.000, 50.000, Point.CARTESIAN),
                new Point(30.000, 29.000, Point.CARTESIAN)));
        alignSpeci3.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        pickupSpeci3 = new Path(new BezierLine(
                new Point(30.000, 29.000, Point.CARTESIAN),
                new Point(25.700, 29.000, Point.CARTESIAN)));
        pickupSpeci3.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreSpeci3 = new Path(new BezierCurve(
                new Point(25.700, 29.000, Point.CARTESIAN),
                new Point(10.000, 40.000, Point.CARTESIAN),
                new Point(20.000, 75.000, Point.CARTESIAN),
                new Point(38.000, 65.000, Point.CARTESIAN)));
        scoreSpeci3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        park = new Path(new BezierCurve(
                new Point(38.000, 65.000, Point.CARTESIAN),
                new Point(20.000, 35.000, Point.CARTESIAN),
                new Point(27.000, 20.000, Point.CARTESIAN)));
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

    public void mSpecimenSet() {
        slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET);
        intake.barNeutral();
        deposit.specimenSetPos();
        deposit.clawShift();
    }

    public void mSlideRest() {
        slides.setTarget(OuttakeSlides.REST_POS);
        deposit.openClaw();
        deposit.depositPos();
    }
}