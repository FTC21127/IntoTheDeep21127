package org.firstinspires.ftc.teamcode.opMode.auton.basket;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous(name = "0 + 4' high", group = "!basket", preselectTeleOp = "Robot")
public class Basket04 extends OpMode {
    Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private Follower follower;
    ElapsedTime gameTimer = new ElapsedTime();

    private Path preload, sample1, basket1, sample2, basket2, sample3, basket3, park;

    private Intake intake = new Intake(this);
    private OuttakeSlides slides = new OuttakeSlides(this);
    private Deposit deposit = new Deposit(this);

    private boolean busy = false, busy2 = false;
    private int SLIDE_ERROR = 10;

    private Command preloadCommand = () -> follower.followPath(preload);
    private Command sample1Command = () -> follower.followPath(sample1, true);
    private Command basketCommand = () -> follower.followPath(basket1, true);
    private Command sample2Command = () -> follower.followPath(sample2, true);
    private Command basket2Command = () -> follower.followPath(basket2, true);
    private Command sample3Command = () -> {follower.followPath(sample3, true);follower.setMaxPower(.7);};
    private Command basket3Command = () -> {follower.followPath(basket3, true);follower.setMaxPower(1);};
    private Command parkCommand = () -> follower.followPath(park);
    private final Command busyTrue = () -> busy = true;
    private final Command busyFalse = () -> busy = false;
    private final Command highSlideError = () -> SLIDE_ERROR = 50;
    private final Command lowSlideError = () -> SLIDE_ERROR = 10;
    private final Command FORCE_STOP = this::requestOpModeStop;

    // Slide Commands
    private final Command slidesIntake = () -> slides.intakePos();
    private final Command intakeRest = () -> slides.setTarget(OuttakeSlides.INTAKE_POS + 20);
    private final Command slideAscend = () -> slides.setTarget(OuttakeSlides.REST_POS + 1200);
    private final Command slideRest = () -> slides.setTarget(OuttakeSlides.REST_POS + 200);
    private final Command highBasket = () -> {slides.setTarget(OuttakeSlides.HIGH_BASKET); deposit.goofyBasketPos();};
    // Deposit Commands
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command basketPos = () -> deposit.basketPos();
    private final Command specimenPos = () -> deposit.specimenSetPos();
    private final Command specimenScorePos = () -> deposit.specimenScorePos();
    private final Command initPos = () -> deposit.initPos();
    private final Command basketSet = () -> deposit.goofyBasketPos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    // Intake Commands
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command dropV4b = () -> intake.autonDown();
    private final Command neutralV4b = () -> intake.barNeutral();
    private final Command transferV4b = () -> intake.barTransfer();

    private CommandSequence initSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addCommand(initPos)
            .addCommand(neutralV4b)
            .addCommand(slideRest)
            .build();

    private CommandSequence move1 = new CommandSequence()
            .addCommand(highSlideError)
            .addCommand(highBasket)
            .addCommand(basketSet)
            .addCommand(preloadCommand)
            .build();
    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(basketPos)
            .addWaitCommand(.1)
            .addCommand(outtakeRelease)
            .addCommand(outtakeRelease)
            .addWaitCommand(.3)
            .addCommand(basketSet)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample1Command)
            .addCommand(dropV4b)
            .addWaitCommand(.5)
            .addCommand(slideRest)
            .addCommand(intakeOpen)
            .addWaitCommand(.3)
            .addCommand(busyFalse)
            .build();
    private CommandSequence transferSequence = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeRest)
            .addWaitCommand(0.1)
            .addCommand(intakeGrab)
            .addWaitCommand(0.5)
            .addCommand(transferV4b)
            .addCommand(outtakeRelease)
            .addCommand(outtakeRelease)
            .addWaitCommand(1)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(neutralV4b)
            .addWaitCommand(.3)
            .addCommand(grabTransfer)
            .addCommand(intakeGrab)
            .addWaitCommand(.1)
            .addCommand(slidesIntake)
            .addWaitCommand(.45)
            .addCommand(outtakeGrab)
            .addCommand(highSlideError)
            .addCommand(outtakeGrab)
            .addWaitCommand(.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move3 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(outtakeGrab)
            .addCommand(basketCommand)
            .addCommand(highSlideError)
            .addCommand(highBasket)
            .addCommand(basketSet)
            .addWaitCommand(.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move4 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample2Command)
            .addCommand(dropV4b)
            .addWaitCommand(.5)
            .addCommand(slideRest)
            .addCommand(intakeOpen)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move5 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(basket2Command)
            .addCommand(highSlideError)
            .addCommand(highBasket)
            .addCommand(basketSet)
            .addWaitCommand(.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move6 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample3Command)
            .addCommand(dropV4b)
            .addWaitCommand(.5)
            .addCommand(slideRest)
            .addCommand(intakeOpen)
            .addWaitCommand(.5)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move7 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(basket3Command)
            .addCommand(highSlideError)
            .addCommand(highBasket)
            .addCommand(basketSet)
            .addWaitCommand(.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move8 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(parkCommand)
            .addWaitCommand(1)
            .addCommand(slideAscend)
            .addCommand(basketPos)
            .addWaitCommand(.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence holdEnd = new CommandSequence()
            .addCommand(lowSlideError)
            .addCommand(busyTrue)
            .addCommand(basketPos)
            .addWaitCommand(.5)
            .addCommand(busyTrue)
            .addCommand(FORCE_STOP)
            .build();

    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(move1)
            .addCommandSequence(scoreBasket)
            .addCommandSequence(move2)
            .addCommandSequence(transferSequence)
            .addCommandSequence(move3)
            .addCommandSequence(scoreBasket)
            .addCommandSequence(move4)
            .addCommandSequence(transferSequence)
            .addCommandSequence(move5)
            .addCommandSequence(scoreBasket)
            .addCommandSequence(move6)
            .addCommandSequence(transferSequence)
            .addCommandSequence(move7)
            .addCommandSequence(scoreBasket)
            .addCommandSequence(move8)
            .addCommandSequence(holdEnd)
            .build();

    @Override
    public void init() {

        intake.init(hardwareMap);
        slides.init(hardwareMap);
        deposit.init(hardwareMap);
        follower = new Follower(hardwareMap);
        slides.reset();

        follower.setPose(new Pose(6.9, 102, Math.toRadians(-90)));

        preload = new Path(new BezierLine(
                new Point(6.9, 102, Point.CARTESIAN),
                new Point(18, 127, Point.CARTESIAN)));
        preload.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-50));
        sample1 = new Path(new BezierLine(
                new Point(18, 126, Point.CARTESIAN),
                new Point(38.5, 126.5, Point.CARTESIAN)));
        sample1.setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(-10));
        basket1 = new Path(new BezierLine(
                new Point(38.5, 126.5, Point.CARTESIAN),
                new Point(20, 126, Point.CARTESIAN)));
        basket1.setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(-45));
        sample2 = new Path(new BezierLine(
                new Point(20, 126, Point.CARTESIAN),
                new Point(39, 134, Point.CARTESIAN)));
        sample2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));
        basket2 = new Path(new BezierLine(
                new Point(39, 134, Point.CARTESIAN),
                new Point(20, 126, Point.CARTESIAN)));
        basket2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
        sample3 = new Path(new BezierCurve(
                new Point(20, 126, Point.CARTESIAN),
                new Point(34, 120, Point.CARTESIAN),
                new Point(40, 140, Point.CARTESIAN)));
        sample3.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(30));
        basket3 = new Path(new BezierLine(
                new Point(40, 140, Point.CARTESIAN),
                new Point(20, 127, Point.CARTESIAN)));
        basket3.setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-45));
        park = new Path(new BezierCurve(
                new Point(20, 127, Point.CARTESIAN),
                new Point(68, 140, Point.CARTESIAN),
                new Point(70, 104 , Point.CARTESIAN)));
        park.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));

        initSequence.trigger();
    }

    @Override
    public void init_loop() {
        slides.update();
        if (gamepad1.cross){
            outtakeGrab.run();
        }
    }

    @Override
    public void start() {
        gameTimer.reset();
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
        busy2 = follower.isBusy() || Math.abs(slides.getError())>SLIDE_ERROR;
        telemetry.addData("is busy", busy);
        telemetry.addData("follower busy? ", busy2);
        telemetry.addData("current command: ", commandMachine.getCurrentCommandIndex());
        telemetry.addData("current error: ", slides.getError());
        telemetry.addData("current pos: ", follower.getPose());
    }
}