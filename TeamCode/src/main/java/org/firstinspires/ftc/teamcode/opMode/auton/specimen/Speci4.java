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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "(don't use) 4 + 0'", group = "!specimen", preselectTeleOp = "Robot")
public class Speci4 extends OpMode {

    Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private Follower follower;

    private Path preload, grabSample1, yeetSample1, pushSample, grabSample2, yeetSample2, alignSpeci2, scoreSpeci2, alignSpeci3, pickupSpeci3, scoreSpeci3, alignSpeci4, pickupSpeci4, scoreSpeci4, park;

    private final Intake intake = new Intake(this);
    private final OuttakeSlides slides = new OuttakeSlides(this);
    private final Deposit deposit = new Deposit(this);

    private boolean busy = false, busy2 = false;

    private final Command preloadCommand = () -> follower.followPath(preload);
    private final Command specimen2Command = () -> follower.followPath(scoreSpeci2, false);
    private final Command specimen3Command = () -> follower.followPath(scoreSpeci3, false);
    private final Command specimen4Command = () -> follower.followPath(scoreSpeci4, false);
    private final Command grabSample1Command = () -> follower.followPath(grabSample1, true);
    private final Command yeetSample1Command = () -> follower.followPath(yeetSample1, true);
    private final Command pushSampleCommand = () -> follower.followPath(pushSample, true);
    private final Command grabSample2Command = () -> follower.followPath(grabSample2, true);
    private final Command yeetSample2Command = () -> follower.followPath(yeetSample2, true);
    private final Command pickupSpecimen3Command = () -> follower.followPath(pickupSpeci3, true);
    private final Command pickupSpecimen4Command = () -> follower.followPath(pickupSpeci4, true);
    private final Command alignSpecimen2Command = () -> follower.followPath(alignSpeci2, true);
    private final Command alignSpecimen3Command = () -> follower.followPath(alignSpeci3, true);
    private final Command alignSpecimen4Command = () -> follower.followPath(alignSpeci4, true);
    private final Command parkCommand = () -> follower.followPath(park, false);

    private final Command busyTrue = () -> busy = true;
    private final Command busyFalse = () -> busy = false;
    private final Command forceQuit = () -> {
        busy2 = false;
        busy = false;
    };

    // Slide Commands
    private final Command slidesIntake = slides::intakePos;
    private final Command slideRest = () -> slides.setTarget(OuttakeSlides.REST_POS + 200);
    private final Command slideSpecimen = () -> slides.setTarget(OuttakeSlides.REST_POS);
    private final Command highSpecimen = () -> slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET - 125);
    private final Command lockSpecimen = slides::lock;
    // Deposit Commands
    private final Command depositPos = deposit::depositPos;
    private final Command grabTransfer = deposit::transferPos;
    private final Command initPos = deposit::initPos;
    private final Command specimenPos = deposit::specimenSetPos;
    private final Command specimenScorePos = deposit::specimenScorePos;
    private final Command depositUp = deposit::goofyBasketPos;
    private final Command outtakeRelease = deposit::openClaw;
    private final Command outtakeGrab = deposit::closeClaw;
    private final Command clawShift = deposit::clawShift;
    // Intake Commands
    private final Command intakeGrab = intake::closeClaw;
    private final Command intakeOpen = intake::openClaw;
    private final Command autonV4b = intake::autonDown;
    private final Command neutralV4b = intake::barNeutral;

    private final CommandSequence initSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addCommand(initPos)
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
    private final CommandSequence sampleGrab1 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(grabSample1Command)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen)
            .addCommand(autonV4b)
            .addWaitCommand(.4)
            .addCommand(depositUp)
            .addCommand(intakeOpen)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence sampleYeet1 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeGrab)
            .addWaitCommand(0.3)
            .addCommand(yeetSample1Command)
            .addCommand(depositPos)
            .addCommand(busyFalse)
            .build();

    private final CommandSequence samplePush = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(pushSampleCommand)
            .addCommand(pushSampleCommand)
            .addWaitCommand(.2)
            .addCommand(intakeGrab)
            .addWaitCommand(.2)
            .addCommand(neutralV4b)
            .addCommand(busyFalse)
            .build();

    private final CommandSequence sampleGrab2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeOpen)
            .addWaitCommand(.3)
            .addCommand(grabSample2Command)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence sampleYeet2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeGrab)
            .addWaitCommand(0.3)
            .addCommand(yeetSample2Command)
            .addCommand(depositPos)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence alignSpecimen2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(alignSpecimen2Command)
            .addCommand(outtakeRelease)
            .addWaitCommand(.3)
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
            .addCommand(specimenPos)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence alignSpecimen3 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(alignSpecimen3Command)
            .addCommand(depositPos)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence pickUpSpecimen3 = new CommandSequence()
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
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence alignSpecimen4 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(alignSpecimen4Command)
            .addCommand(depositPos)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence pickUpSpecimen4 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(pickupSpecimen4Command)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen) // just in case
            .addCommand(depositPos)
            .addCommand(outtakeRelease)
            .addCommand(busyFalse)
            .build();
    private final CommandSequence speci4 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.2)
            .addCommand(specimenPos)
            .addWaitCommand(.1)
            .addCommand(specimen4Command)
            .addCommand(highSpecimen)
            .addCommand(highSpecimen)
            .addCommand(specimenPos)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addWaitCommand(2)
            .addCommand(forceQuit)
            .addCommand(busyFalse)
            .build();

    private final CommandSequence holdEnd = new CommandSequence()
            .addCommand(parkCommand)
            .addCommand(slideRest)
            .addCommand(busyTrue)
            .addCommand(busyTrue)
            .build();

    private final AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(specimen1)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(sampleGrab1)
            .addCommandSequence(sampleYeet1)
            .addCommandSequence(samplePush)
//            .addCommandSequence(sampleGrab2)
//            .addCommandSequence(sampleYeet2)
            .addCommandSequence(alignSpecimen2)
            .addCommandSequence(speci2)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(alignSpecimen3)
            .addCommandSequence(pickUpSpecimen3)
            .addCommandSequence(speci3)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(alignSpecimen4)
//            .addCommandSequence(pickUpSpecimen4)
            .addCommandSequence(speci4)
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
                new Point(37.00, 61.000, Point.CARTESIAN)));
        preload.setConstantHeadingInterpolation(Math.toRadians(180));

        grabSample1 = new Path(new BezierCurve(
                new Point(37.00, 61.000, Point.CARTESIAN),
                new Point(30.000, 42.000, Point.CARTESIAN),
                new Point(47.500, 26.000, Point.CARTESIAN)));
        grabSample1.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-40));

        yeetSample1 = new Path(new BezierLine(
                new Point(47.500, 26.000, Point.CARTESIAN),
                new Point(26.000, 26.000, Point.CARTESIAN)));
        yeetSample1.setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-120));

        pushSample = new Path(new BezierCurve(
                new Point(26.000, 26.000, Point.CARTESIAN),
                new Point(76.000, 30.000, Point.CARTESIAN),
                new Point(76.000, 5.000, Point.CARTESIAN),
                new Point(18.000, 18.000, Point.CARTESIAN)));
        pushSample.setConstantHeadingInterpolation(Math.toRadians(180));

        grabSample2 = new Path(new BezierLine(
                new Point(26.00, 20.000, Point.CARTESIAN),
                new Point(47.00, 15.00, Point.CARTESIAN)));
        grabSample2.setConstantHeadingInterpolation(Math.toRadians(-40));
        yeetSample2 = new Path(new BezierLine(
                new Point(47.00, 15.00, Point.CARTESIAN),
                new Point(27.000, 19.000, Point.CARTESIAN)));
        yeetSample2.setConstantHeadingInterpolation(Math.toRadians(-130));

        alignSpeci2 = new Path(new BezierCurve(
                new Point(18.000, 18.000, Point.CARTESIAN),
                new Point(35.000, 20.000, Point.CARTESIAN),
                new Point(27.000, 20.000, Point.CARTESIAN)));
        alignSpeci2.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreSpeci2 = new Path(new BezierCurve(
                new Point(27.000, 20.000, Point.CARTESIAN),
                new Point(14.000, 40.000, Point.CARTESIAN),
                new Point(22.000, 70.000, Point.CARTESIAN),
                new Point(38.000, 66.000, Point.CARTESIAN)));
        scoreSpeci2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        alignSpeci3 = new Path(new BezierCurve(
                new Point(38.000, 66.000, Point.CARTESIAN),
                new Point(18.000, 60.000, Point.CARTESIAN),
                new Point(30.000, 29.000, Point.CARTESIAN)));
        alignSpeci3.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        pickupSpeci3 = new Path(new BezierLine(
                new Point(30.000, 29.000, Point.CARTESIAN),
                new Point(27.000, 29.000, Point.CARTESIAN)));
        pickupSpeci3.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreSpeci3 = new Path(new BezierCurve(
                new Point(27.000, 29.000, Point.CARTESIAN),
                new Point(14.000, 40.000, Point.CARTESIAN),
                new Point(22.000, 70.000, Point.CARTESIAN),
                new Point(38.000, 66.000, Point.CARTESIAN)));
        scoreSpeci3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        alignSpeci4 = new Path(new BezierLine(
                new Point(38.000, 66.000, Point.CARTESIAN),
                new Point(26.000, 29.000, Point.CARTESIAN)));
        alignSpeci4.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        pickupSpeci4 = new Path(new BezierLine(
                new Point(30.000, 29.000, Point.CARTESIAN),
                new Point(26.000, 29.000, Point.CARTESIAN)));
        pickupSpeci4.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreSpeci4 = new Path(new BezierCurve(
                new Point(26.000, 29.000, Point.CARTESIAN),
                new Point(14.000, 40.000, Point.CARTESIAN),
                new Point(22.000, 70.000, Point.CARTESIAN),
                new Point(38.000, 65.000, Point.CARTESIAN)));
        scoreSpeci4.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        park = new Path(new BezierCurve(
                new Point(38.000, 65.000, Point.CARTESIAN),
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
