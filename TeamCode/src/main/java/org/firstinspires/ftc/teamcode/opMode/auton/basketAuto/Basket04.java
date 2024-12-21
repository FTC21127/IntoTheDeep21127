package org.firstinspires.ftc.teamcode.opMode.auton.basketAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.fissionlib.command.AutoCommandMachine;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlides;

@Autonomous(name = "0 + 4' high", group = "!basket", preselectTeleOp = "Robot")
public class Basket04 extends OpMode {

    Intake diffy = new Intake(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    Deposit outtake = new Deposit(this);
    Follower base;

    Pose START_POSE = new Pose(7.5, 112, Math.toRadians(-90));
    Path preload, sample1, basket1, sample2, basket2, sample3, basket3;

    boolean busy;

    Command preloadCommand = () -> base.followPath(preload);
    Command sample1Command = () -> base.followPath(sample1);
    Command basket1Command = () -> base.followPath(basket1);
    Command sample2Command = () -> base.followPath(sample2);
    Command basket2Command = () -> base.followPath(basket2);
    Command sample3Command = () -> base.followPath(sample3);
    Command basket3Command = () -> base.followPath(basket3);

    Command extendIntake = this::samplePickUp;
    Command extendIntakeSideWays = this::samplePickUp3rd;
    Command retractIntake = this::grabSample;
    Command depositTransfer = this::transferRelease;
    Command outtakeReset = this::retractOuttake;

    Command closeIntake = () -> diffy.closeClaw();
    Command closeOuttake = () -> outtake.closeClaw();
    Command openOuttake = () -> outtake.openClaw();

    CommandSequence movePreload = new CommandSequence()
            .addCommand(preloadCommand)
            .addCommand(depositTransfer)
            .addWaitCommand(0.5)
            .addCommand(extendIntake)
            .build();
    CommandSequence scorePreload = new CommandSequence()
            .addCommand(openOuttake)
            .addWaitCommand(0.5)
            .addCommand(sample1Command)
            .addWaitCommand(.1)
            .addCommand(outtakeReset)
            .build();
    CommandSequence pickUp1 = new CommandSequence()
            .addCommand(closeIntake)
            .addWaitCommand(.1)
            .addCommand(retractIntake)
            .build();
    CommandSequence move1 = new CommandSequence()
            .addCommand(basket1Command)
            .addWaitCommand(.2)
            .addCommand(closeOuttake)
            .addWaitCommand(0.1)
            .addCommand(depositTransfer)
            .addCommand(extendIntake)
            .build();
    CommandSequence score1 = new CommandSequence()
            .addCommand(openOuttake)
            .addWaitCommand(0.5)
            .addCommand(sample2Command)
            .addWaitCommand(.1)
            .addCommand(outtakeReset)
            .build();
    CommandSequence pickUp2 = new CommandSequence()
            .addCommand(closeIntake)
            .addWaitCommand(.1)
            .addCommand(retractIntake)
            .build();
    CommandSequence move2 = new CommandSequence()
            .addCommand(basket2Command)
            .addWaitCommand(.2)
            .addCommand(closeOuttake)
            .addWaitCommand(0.1)
            .addCommand(depositTransfer)
            .addCommand(extendIntakeSideWays)
            .build();
    CommandSequence score2 = new CommandSequence()
            .addCommand(openOuttake)
            .addWaitCommand(0.5)
            .addCommand(sample3Command)
            .addWaitCommand(.1)
            .addCommand(outtakeReset)
            .build();
    CommandSequence pickUp3 = new CommandSequence()
            .addCommand(closeIntake)
            .addWaitCommand(.1)
            .addCommand(retractIntake)
            .build();
    CommandSequence move3 = new CommandSequence()
            .addCommand(basket3Command)
            .addWaitCommand(.2)
            .addCommand(closeOuttake)
            .addWaitCommand(0.1)
            .addCommand(depositTransfer)
            .build();
    CommandSequence score3 = new CommandSequence()
            .addCommand(openOuttake)
            .addWaitCommand(0.5)
            .addCommand(outtakeReset)
            .build();

    AutoCommandMachine autoMachine = new AutoCommandMachine()
            .addCommandSequence(movePreload)
            .addCommandSequence(scorePreload)
            .addCommandSequence(pickUp1)
            .addCommandSequence(move1)
            .addCommandSequence(score1)
            .addCommandSequence(pickUp2)
            .addCommandSequence(move2)
            .addCommandSequence(score2)
            .addCommandSequence(pickUp3)
            .addCommandSequence(move3)
            .addCommandSequence(score3)
            .build();

    @Override
    public void init() {
        diffy.init(hardwareMap);
        slides.init(hardwareMap);
        outtake.init(hardwareMap);
        base = new Follower(hardwareMap);
        base.setPose(START_POSE);

        preload = new Path(new BezierLine(
                new Point(7.500, 112.000, Point.CARTESIAN),
                new Point(14, 127.5, Point.CARTESIAN)));
        preload.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));
        sample1 = new Path(new BezierLine(
                new Point(14, 127.5, Point.CARTESIAN),
                new Point(24.000, 120.000, Point.CARTESIAN)));
        sample1.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));
        basket1 = new Path(new BezierLine(
                new Point(24.000, 120.000, Point.CARTESIAN),
                new Point(14.361, 128.262, Point.CARTESIAN)));
        basket1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
        sample2 = new Path(new BezierLine(
                new Point(14.361, 128.262, Point.CARTESIAN),
                new Point(24.000, 130.000, Point.CARTESIAN)));
        sample2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));
        basket2 = new Path(new BezierLine(
                new Point(24.000, 130.000, Point.CARTESIAN),
                new Point(14.361, 128.066, Point.CARTESIAN)));
        basket2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
        sample3 = new Path(new BezierCurve(
                new Point(14.361, 128.066, Point.CARTESIAN),
                new Point(24.000, 110.164, Point.CARTESIAN),
                new Point(45.639, 120.590, Point.CARTESIAN)));
        sample3.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));
        basket3 = new Path(new BezierLine(
                new Point(45.639, 120.590, Point.CARTESIAN),
                new Point(14.361, 128.262, Point.CARTESIAN)));
        basket3.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-45));

        diffy.diffyNeutral();
        outtake.transferPos();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a){
            outtake.closeClaw();
        }
    }

    @Override
    public void start() {
        autoMachine.run(busy);
        base.update();
        slides.update();
        busy = true;
    }

    @Override
    public void loop() {
        autoMachine.run(busy);
        base.update();
        slides.update();
        busy = base.isBusy() || !slides.isDone();
        telemetry.addData("is busy", busy);
        telemetry.addData("follower busy? ", busy);
        telemetry.addData("current command: ", autoMachine.getCurrentCommandIndex());
        telemetry.addData("current error: ", slides.getError());
    }
    
    public void samplePickUp(){
        diffy.extendNeutral();
        diffy.diffyDown();
        diffy.openClaw();
    }
    
    public void samplePickUp3rd(){
        diffy.extendNeutral();
        diffy.diffyDown();
        diffy.openClaw();
        diffy.setDif_ROLL(90);
    }

    public void grabSample(){
        diffy.diffyTransfer();
        diffy.shiftClaw();
        diffy.retractExtendy();
        slides.intakePos();
        outtake.transferPos();
        outtake.openClaw();
    }

    public void transferRelease(){
        diffy.openClaw();
        diffy.diffyNeutral();
        outtake.basketPos();
        slides.setTarget(OuttakeSlides.HIGH_BASKET);
    }
    
    public void retractOuttake(){
        outtake.transferPos();
        slides.intakePos();
    }
}
