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


@Autonomous(name = "2 + 0'", group = "!specimen", preselectTeleOp = "Robot")
public class Speci2 extends OpMode {

    Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    Timing.Timer pickupTimer = new Timing.Timer(1500, TimeUnit.MILLISECONDS);
    Timing.Timer ScoringTimer = new Timing.Timer(2500, TimeUnit.MILLISECONDS);
    private Follower follower;

    private Path specimen1, specimen2, pickUpSpecimen, alignS1, pushS1, alignS2, pushS2, alignSpecimen, park;

    private Intake intake = new Intake(this);
    private OuttakeSlides slides = new OuttakeSlides(this);
    private Deposit deposit = new Deposit(this);

    private boolean busy = false, busy2 = false;
    private int SLIDE_ERROR = 10;

    private Command specimen1Command = () -> follower.followPath(specimen1);
    private Command specimen2Command = () -> follower.followPath(specimen2, true);
    private Command pickUpCommand = () -> follower.followPath(pickUpSpecimen, true);
    private Command alignS1Command = () -> {follower.followPath(alignS1, true);follower.setMaxPower(.7);};
    private Command alignS2Command = () -> follower.followPath(alignS2, true);
    private Command alignSpecimenCommand = () -> follower.followPath(alignSpecimen, true);
    private Command pushS1Command = () -> {follower.followPath(pushS1, true);follower.setMaxPower(1);};
    private Command pushS2Command = () -> follower.followPath(pushS2, true);

    private final Command busyTrue = () -> busy = true;
    private final Command busyFalse = () -> busy = false;
    private final Command pickupStart = () -> pickupTimer.start();
    private final Command scoreStart = () -> ScoringTimer.start();
    private final Command forceQuit = () -> {busy2 = false;busy = false;};

    // Slide Commands
    private final Command slideRest = () -> slides.setTarget(OuttakeSlides.REST_POS + 200);
    private final Command slideSpecimen = () -> slides.setTarget(OuttakeSlides.REST_POS);
    private final Command highSpecimen = () -> slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET - 115);
    private final Command lockSpecimen = () -> slides.lock();
    private final Command atPos = () -> slides.isDone();
    // Deposit Commands
    private final Command depositPos = () -> deposit.depositPos();
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command specimenPos = () -> deposit.specimenSetPos();
    private final Command specimenScorePos = () -> deposit.specimenScorePos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    private final Command clawShift = () -> deposit.clawShift();
    // Intake Commands
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command dropV4b = () -> intake.barDown();
    private final Command neutralV4b = () -> intake.barNeutral();

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
            .addCommand(specimen1Command)
            .build();
    private CommandSequence scoreSpecimen = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(0.2)
            .addCommand(specimenScorePos)
            .addCommand(specimenScorePos)
            .addWaitCommand(.4)
            .addCommand(lockSpecimen)
            .addWaitCommand(0.2)
            .addCommand(outtakeRelease)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2a = new CommandSequence()
            .addCommand(alignS1Command)
            .addWaitCommand(.3)
            .addCommand(depositPos)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen)
            .build();
    private CommandSequence move2b = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(pushS1Command)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2c = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(alignS2Command)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2d = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(pushS2Command)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2e = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(alignSpecimenCommand)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move3 = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(1)
            .addCommand(pickUpCommand)
            .addWaitCommand(.5)
            .addCommand(pickupStart)
            .addCommand(pickupStart)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move4 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.5)
            .addCommand(specimen2Command)
            .addCommand(highSpecimen)
            .addCommand(highSpecimen)
            .addCommand(specimenPos)
            .addCommand(specimenPos)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(scoreStart)
            .addWaitCommand(2)
            .addCommand(forceQuit)
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
            .addCommandSequence(move2a)
            .addCommandSequence(move2b)
            .addCommandSequence(move2c)
            .addCommandSequence(move2d)
            .addCommandSequence(move2e)
            .addCommandSequence(move3)
            .addCommandSequence(move4)
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

        follower.setPose(new Pose(7.25, 64, Math.toRadians(180)));

        specimen1 = new Path(new BezierLine(
                new Point(7.250, 64.000, Point.CARTESIAN),
                new Point(36.000, 65.000, Point.CARTESIAN)));
        specimen1.setConstantHeadingInterpolation(Math.toRadians(180));

        alignS1 = new Path(new BezierCurve(
                new Point(36.000, 65.000, Point.CARTESIAN),
                new Point(21.995, 31.394, Point.CARTESIAN),
                new Point(60.721, 38.470, Point.CARTESIAN),
                new Point(60.000, 24.000, Point.CARTESIAN)));

        alignS1.setConstantHeadingInterpolation(Math.toRadians(180));

        pushS1 = new Path(new BezierLine(
                new Point(60.000, 24.000, Point.CARTESIAN),
                new Point(18.000, 24.000, Point.CARTESIAN)));
        pushS1.setConstantHeadingInterpolation(Math.toRadians(180));

        alignS2 = new Path(new BezierCurve(
                new Point(18.000, 24.000, Point.CARTESIAN),
                new Point(64.104, 26.507, Point.CARTESIAN),
                new Point(58.000, 13.000, Point.CARTESIAN)));
        alignS2.setConstantHeadingInterpolation(Math.toRadians(180));

        pushS2 = new Path(new BezierLine(
                new Point(58.000, 13.000, Point.CARTESIAN),
                new Point(17.000, 13.000, Point.CARTESIAN)));
        pushS2.setConstantHeadingInterpolation(Math.toRadians(180));

        alignSpecimen = new Path(new BezierCurve(
                new Point(17.000, 13.000, Point.CARTESIAN),
                new Point(40, 25, Point.CARTESIAN),
                new Point(36.000, 32.000, Point.CARTESIAN)));
        alignSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));

        pickUpSpecimen = new Path(new BezierLine(
                new Point(36, 32, Point.CARTESIAN),
                new Point(25, 32, Point.CARTESIAN)));
        pickUpSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));

        specimen2 = new Path(new BezierCurve(
                new Point(25, 32, Point.CARTESIAN),
                new Point(5, 50, Point.CARTESIAN),
                new Point(37.75, 63, Point.CARTESIAN)));
        specimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        park = new Path(new BezierCurve(
                new Point(37.75, 63, Point.CARTESIAN),
                new Point(5, 50, Point.CARTESIAN),
                new Point(25, 32, Point.CARTESIAN)));
        park.setTangentHeadingInterpolation();
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
        if (commandMachine.getCurrentCommandIndex()==8 && pickupTimer.done()){
            busy2 = false;
        } else if (commandMachine.getCurrentCommandIndex()==9 && ScoringTimer.done()) {
            busy2 = false;
        }
        commandMachine.run(busy2 || busy);
        follower.update();
        slides.update();
        busy2 = follower.isBusy() || Math.abs(slides.getError()) > SLIDE_ERROR;
        telemetry.addData("is busy", busy);
        telemetry.addData("follower busy? ", busy2);
        if (pickupTimer.isTimerOn()) {
            telemetry.addData("pickup timer ", pickupTimer.elapsedTime() | ScoringTimer.elapsedTime());
        }
        telemetry.addData("current command: ", commandMachine.getCurrentCommandIndex());
        telemetry.addData("current error: ", slides.getError());
        telemetry.addData("current pos: ", follower.getPose().getX());
    }
}