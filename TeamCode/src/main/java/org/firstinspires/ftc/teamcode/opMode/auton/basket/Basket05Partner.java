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


@Autonomous(name = "0 + 5' partner", group = "!basket", preselectTeleOp = "Robot")
public class Basket05Partner extends OpMode {
    Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private Follower follower;
    ElapsedTime gameTimer = new ElapsedTime();

    private Path preload, sample1, basket1, sample2, basket2, sample3, basket3, sample4, basket4, park;

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
    private Command sample4Command = () -> follower.followPath(sample4, true);
    private Command basket4Command = () -> follower.followPath(basket4, true);
    private Command parkCommand = () -> follower.followPath(park);
    private final Command busyTrue = () -> busy = true;
    private final Command busyFalse = () -> busy = false;
    private final Command basketSet = this::mBasketSet;

    private final Command slideAscend = () -> slides.setTarget(OuttakeSlides.REST_POS + 1200);
    private final Command slideRest = () -> slides.restPos();
    // Deposit Commands
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command basketPos = () -> deposit.basketPos();
    private final Command initPos = () -> deposit.initPos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    // Intake Commands
    private final Command intakeGrab = () -> intake.closeClaw();
    private final Command intakeOpen = () -> intake.openClaw();
    private final Command neutralV4b = () -> intake.barNeutral();
    private final Command autonV4B = () -> intake.autonDown();
    private final Command transferV4b = () -> intake.barTransfer();

    private CommandSequence initSequence = new CommandSequence()
            .addCommand(intakeGrab)
            .addCommand(initPos)
            .addCommand(neutralV4b)
            .addCommand(slideRest)
            .build();

    private CommandSequence move1 = new CommandSequence()
            .addCommand(basketSet)
            .addCommand(preloadCommand)
            .build();
    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(basketPos)
            .addWaitCommand(0.09)
            .addCommand(outtakeRelease)
            .addCommand(outtakeRelease)
            .addWaitCommand(.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move2 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample1Command)
            .addCommand(this::mIntakeExtend)
            .addWaitCommand(.2)
            .addCommand(intakeOpen)
            .addCommand(busyFalse)
            .build();
    private CommandSequence transferSequence = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeGrab)
            .addWaitCommand(0.4)
            .addCommand(transferV4b)
            .addWaitCommand(1)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(neutralV4b)
            .addWaitCommand(.3)
            .addCommand(grabTransfer)
            .addCommand(intakeGrab)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addCommand(outtakeGrab)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move3 = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(0.1)
            .addCommand(basketCommand)
            .addCommand(basketSet)
            .addWaitCommand(0.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move4 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample2Command)
            .addCommand(this::mIntakeExtend)
            .addWaitCommand(.2)
            .addCommand(intakeOpen)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move5 = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(0.1)
            .addCommand(basket2Command)
            .addCommand(basketSet)
            .addWaitCommand(0.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move6 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample3Command)
            .addCommand(this::mIntakeExtend)
            .addWaitCommand(.2)
            .addCommand(intakeOpen)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move7 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeGrab)
            .addCommand(autonV4B)
            .addWaitCommand(0.3)
            .addCommand(transferV4b)
            .addWaitCommand(1)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(neutralV4b)
            .addWaitCommand(.3)
            .addCommand(grabTransfer)
            .addCommand(intakeGrab)
            .addCommand(basket3Command)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addWaitCommand(.2)
            .addCommand(basketSet)
            .addWaitCommand(0.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move8 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(sample4Command)
            .addCommand(this::mIntakeExtend)
            .addWaitCommand(.2)
            .addCommand(intakeOpen)
            .addCommand(busyFalse)
            .build();
    private CommandSequence move9 = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(intakeGrab)
            .addWaitCommand(0.1)
            .addCommand(autonV4B)
            .addWaitCommand(0.2)
            .addCommand(transferV4b)
            .addWaitCommand(1)
            .addCommand(intakeOpen)
            .addWaitCommand(.2)
            .addCommand(neutralV4b)
            .addWaitCommand(.3)
            .addCommand(grabTransfer)
            .addCommand(intakeGrab)
            .addCommand(basket4Command)
            .addWaitCommand(.5)
            .addCommand(outtakeGrab)
            .addWaitCommand(.2)
            .addCommand(basketSet)
            .addWaitCommand(0.1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence holdEnd = new CommandSequence()
            .addCommand(busyTrue)
            .addCommand(parkCommand)
            .addCommand(this::mSlideRest)
            .addWaitCommand(.3)
            .addCommand(transferV4b)
            .addCommand(slideAscend)
            .addWaitCommand(.3)
            .addCommand(basketPos)
            .addCommand(busyTrue)
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
            .addCommandSequence(move7)
            .addCommandSequence(scoreBasket)
            .addCommandSequence(move8)
            .addCommandSequence(move9)
            .addCommandSequence(scoreBasket)
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
                new Point(20, 126.5, Point.CARTESIAN)));
        preload.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-50));
        sample1 = new Path(new BezierLine(
                new Point(20, 126.5, Point.CARTESIAN),
                new Point(39, 126.5, Point.CARTESIAN)));
        sample1.setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(-10));
        basket1 = new Path(new BezierLine(
                new Point(38, 126.5, Point.CARTESIAN),
                new Point(21, 127, Point.CARTESIAN)));
        basket1.setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(-45));
        sample2 = new Path(new BezierLine(
                new Point(21, 127, Point.CARTESIAN),
                new Point(39, 134.5, Point.CARTESIAN)));
        sample2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));
        basket2 = new Path(new BezierLine(
                new Point(39, 134.5, Point.CARTESIAN),
                new Point(20.5, 127, Point.CARTESIAN)));
        basket2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
        sample3 = new Path(new BezierCurve(
                new Point(20.5, 127, Point.CARTESIAN),
                new Point(34, 120, Point.CARTESIAN),
                new Point(40, 140.5, Point.CARTESIAN)));
        sample3.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(30));
        basket3 = new Path(new BezierLine(
                new Point(40, 140.5, Point.CARTESIAN),
                new Point(21.5, 129, Point.CARTESIAN)));
        basket3.setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-45));
        sample4 = new Path(new BezierLine(
                new Point(22, 129, Point.CARTESIAN),
                new Point(14.6, 85, Point.CARTESIAN)));
        sample4.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-96));
        basket4 = new Path(new BezierLine(
                new Point(13.6, 85, Point.CARTESIAN),
                new Point(22, 127, Point.CARTESIAN)));
        basket4.setLinearHeadingInterpolation(Math.toRadians(-96), Math.toRadians(-45));
        park = new Path(new BezierCurve(
                new Point(22, 127, Point.CARTESIAN),
                new Point(70, 115, Point.CARTESIAN),
                new Point(70, 104 , Point.CARTESIAN)));
        park.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));

        initSequence.trigger();
    }

    @Override
    public void init_loop() {
        slides.update();
        if (gamepad1.cross){
            deposit.closeClaw();
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

    public void mIntakeExtend() {
        intake.barDown();
        mSlideRest();
    }

    public void mBasketSet() {
        slides.setTarget(OuttakeSlides.HIGH_BASKET);
        intake.barNeutral();
        deposit.goofyBasketPos();
    }

    public void mSlideRest() {
        slides.setTarget(OuttakeSlides.INTAKE_POS+10);
        deposit.openClaw();
        deposit.goofyBasketPos();
    }
}