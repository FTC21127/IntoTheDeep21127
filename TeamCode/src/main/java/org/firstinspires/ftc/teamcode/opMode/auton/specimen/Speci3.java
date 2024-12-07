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
    Timing.Timer pickupTimer = new Timing.Timer(1500, TimeUnit.MILLISECONDS);
    Timing.Timer ScoringTimer = new Timing.Timer(2500, TimeUnit.MILLISECONDS);
    private Follower follower;

    private Path specimen1, specimen2, specimen3, pickUpSpecimen, alignS1, pushS1, alignSpecimen1, alignSpecimen2, park;

    private Intake intake = new Intake(this);
    private OuttakeSlides slides = new OuttakeSlides(this);
    private Deposit deposit = new Deposit(this);

    private boolean busy = false, busy2 = false;
    private int SLIDE_ERROR = 10;

    private Command specimen1Command = () -> follower.followPath(specimen1);
    private Command specimen2Command = () -> follower.followPath(specimen2, false);
    private Command specimen3Command = () -> follower.followPath(specimen3, false);
    private Command pickUpCommand = () -> follower.followPath(pickUpSpecimen, true);
    private Command alignS1Command = () -> {follower.followPath(alignS1, true);follower.setMaxPower(.7);};
    private Command alignSpecimen1Command = () -> follower.followPath(alignSpecimen1, true);
    private Command alignSpecimen2Command = () -> follower.followPath(alignSpecimen2, true);
    private Command pushS1Command = () -> {follower.followPath(pushS1, true);follower.setMaxPower(1);};

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

    private CommandSequence speci1 = new CommandSequence()
            .addCommand(highSpecimen)
            .addCommand(outtakeGrab)
            .addCommand(specimenPos)
            .addCommand(specimen1Command)
            .addWaitCommand(.2)
            .build();
    private CommandSequence scoreSpecimen = new CommandSequence()
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
    private CommandSequence push1a = new CommandSequence()
            .addCommand(alignS1Command)
            .addWaitCommand(.3)
            .addCommand(depositPos)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen)
            .build();
    private CommandSequence push1b = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(pushS1Command)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence sp1Align = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(alignSpecimen1Command)
            .addWaitCommand(.2)
            .addCommand(busyFalse)
            .build();
    private CommandSequence sp2Align = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(depositPos)
            .addWaitCommand(.3)
            .addCommand(slideSpecimen)
            .addCommand(alignSpecimen2Command)
            .addWaitCommand(.2)
            .addCommand(busyFalse)
            .build();
    private CommandSequence speci2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.2)
            .addCommand(specimenPos)
            .addWaitCommand(.3)
            .addCommand(specimen2Command)
            .addCommand(highSpecimen)
            .addCommand(highSpecimen)
            .addCommand(specimenPos)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(scoreStart)
            .addWaitCommand(2)
            .addCommand(forceQuit)
            .addCommand(busyFalse)
            .build();
    private CommandSequence speci3 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(clawShift)
            .addWaitCommand(.2)
            .addCommand(specimenPos)
            .addWaitCommand(.3)
            .addCommand(specimen3Command)
            .addCommand(highSpecimen)
            .addCommand(highSpecimen)
            .addCommand(specimenPos)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(scoreStart)
            .addWaitCommand(2)
            .addCommand(forceQuit)
            .addCommand(busyFalse)
            .build();
    private CommandSequence pickup = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(.7)
            .addCommand(pickUpCommand)
            .addWaitCommand(.2)
            .addCommand(pickupStart)
            .addCommand(pickupStart)
            .addCommand(busyFalse)
            .build();
    private CommandSequence holdEnd = new CommandSequence()
            .addCommand(slideRest)
            .addCommand(busyTrue)
            .addCommand(busyTrue)
            .build();

    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(speci1)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(push1a)
            .addCommandSequence(push1b)
            .addCommandSequence(sp1Align)
            .addCommandSequence(pickup)
            .addCommandSequence(speci2)
            .addCommandSequence(scoreSpecimen)
            .addCommandSequence(sp2Align)
            .addCommandSequence(pickup)
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

        follower.setPose(new Pose(7.25, 64, Math.toRadians(180)));

        specimen1 = new Path(new BezierLine(
                new Point(7.250, 64.000, Point.CARTESIAN),
                new Point(36.000, 65.000, Point.CARTESIAN)));
        specimen1.setConstantHeadingInterpolation(Math.toRadians(180));

        alignS1 = new Path(new BezierCurve(
                new Point(36.000, 64.000, Point.CARTESIAN),
                new Point(22.371, 28.198, Point.CARTESIAN),
                new Point(70.000, 38.000, Point.CARTESIAN),
                new Point(70.000, 24.000, Point.CARTESIAN)));
        alignS1.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        pushS1 = new Path(new BezierLine(
                new Point(70.000, 24.000, Point.CARTESIAN),
                new Point(28.000, 24.000, Point.CARTESIAN)));
        pushS1.setConstantHeadingInterpolation(Math.toRadians(0));
        pushS1.setReversed(true);

        alignSpecimen1 = new Path(new BezierLine(
                new Point(28.000, 24.000, Point.CARTESIAN),
                new Point(34.000, 32.000, Point.CARTESIAN)));
        alignSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));

        pickUpSpecimen = new Path(new BezierLine(
                new Point(34.000, 32.000, Point.CARTESIAN),
                new Point(24.000, 32.000, Point.CARTESIAN)));
        pickUpSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));

        specimen2 = new Path(new BezierCurve(
                new Point(24.000, 32.000, Point.CARTESIAN),
                new Point(10.715, 56.773, Point.CARTESIAN),
                new Point(37.500, 62.000, Point.CARTESIAN)));
        specimen2.setConstantHeadingInterpolation(Math.toRadians(180));

        alignSpecimen2 = new Path(new BezierCurve(
                new Point(37.500, 62.000, Point.CARTESIAN),
                new Point(10, 70, Point.CARTESIAN),
                new Point(34.000, 32.000, Point.CARTESIAN)));
        alignSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        specimen3 = new Path(new BezierCurve(
                new Point(40.000, 32.000, Point.CARTESIAN),
                new Point(11.843, 57.713, Point.CARTESIAN),
                new Point(42.000, 61.000, Point.CARTESIAN)));
        specimen3.setConstantHeadingInterpolation(Math.toRadians(180));

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
        if ((commandMachine.getCurrentCommandIndex()==6 || commandMachine.getCurrentCommandIndex()==10) && pickupTimer.done()){
            busy2 = false;
        } else if ((commandMachine.getCurrentCommandIndex()==7 || commandMachine.getCurrentCommandIndex()==11) && ScoringTimer.done()) {
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