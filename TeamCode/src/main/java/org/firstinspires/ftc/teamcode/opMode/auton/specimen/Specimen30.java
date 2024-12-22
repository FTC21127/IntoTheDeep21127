package org.firstinspires.ftc.teamcode.opMode.auton.specimen;

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


@Autonomous(name = "3 + 0'", group = "!specimen", preselectTeleOp = "Robot")
public class Specimen30 extends OpMode {

    Intake diffy = new Intake(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    Deposit outtake = new Deposit(this);
    Follower base;

    Pose START_POSE = new Pose(7, 54.5, Math.toRadians(180));
    Path preload, sample1Align, sample1Push, sample2Align, sample2Push, specimen1Align, specimen1Score, specimen2Align, specimen2Score;

    boolean busy = false;
    double extendyPos = 0;

    Command preloadCommand = () -> base.followPath(preload);
    Command sample1AlignCommand = () -> base.followPath(sample1Align);
    Command sample1PushCommand = () -> base.followPath(sample1Push);
    Command sample2AlignCommand = () -> base.followPath(sample2Align);
    Command sample2PushCommand = () -> base.followPath(sample2Push);
    Command specimen1AlignCommand = () -> base.followPath(specimen1Align);
    Command specimen1ScoreCommand = () -> base.followPath(specimen1Score);
    Command specimen2AlignCommand = () -> base.followPath(specimen2Align);
    Command specimen2ScoreCommand = () -> base.followPath(specimen2Score);

    Command openIntake = () -> {diffy.openClaw();diffy.diffySearch();};
    Command closeOuttake = () -> outtake.closeClaw();
    Command grabSample = () -> {diffy.shiftClaw(); diffy.diffyDown();};

    CommandSequence movepreload = new CommandSequence()
            .addCommand(preloadCommand)
            .addCommand(this::highChamber)
            .build();
    CommandSequence scoreSpecimen1 = new CommandSequence()
            .addCommand(this::scoreSpecimen)
            .addWaitCommand(.4)
            .addCommand(sample1AlignCommand)
            .addWaitCommand(.1)
            .addCommand(this::releaseSpecimen)
            .addWaitCommand(.2)
            .addCommand(this::extendDiffyPush)
            .build();
    CommandSequence pushSample1 = new CommandSequence()
            .addCommand(grabSample)
            .addWaitCommand(.1)
            .addCommand(sample1PushCommand)
            .addWaitCommand(.1)
            .build();
    CommandSequence alignSample2 = new CommandSequence()
            .addCommand(openIntake)
            .addWaitCommand(.1)
            .addCommand(sample2AlignCommand)
            .addWaitCommand(.1)
            .build();
    CommandSequence pushSample2 = new CommandSequence()
            .addCommand(grabSample)
            .addWaitCommand(.1)
            .addCommand(sample2PushCommand)
            .addWaitCommand(.1)
            .build();
    CommandSequence alignSpecimen2 = new CommandSequence()
            .addCommand(openIntake)
            .addWaitCommand(.1)
            .addCommand(specimen1AlignCommand)
            .addCommand(this::retractDiffy)
            .addWaitCommand(.1)
            .addCommand(this::extendDiffy)
            .build();
    CommandSequence grabSpecimen2 = new CommandSequence()
            .addCommand(grabSample)
            .addWaitCommand(.1)
            .addCommand(this::retractDiffy)
            .addCommand(specimen1ScoreCommand)
            .addWaitCommand(.2)
            .addCommand(this::rearrangeSpecimen)
            .addWaitCommand(.5)
            .addCommand(closeOuttake)
            .addWaitCommand(.1)
            .addCommand(this::highChamber)
            .build();
    CommandSequence scoreSpecimen2 = new CommandSequence()
            .addCommand(this::scoreSpecimen)
            .addWaitCommand(.4)
            .addCommand(specimen2AlignCommand)
            .addWaitCommand(.1)
            .addCommand(this::releaseSpecimen)
            .addCommand(this::extendDiffy)
            .build();
    CommandSequence grabSpecimen3 = new CommandSequence()
            .addCommand(grabSample)
            .addWaitCommand(.1)
            .addCommand(this::retractDiffy)
            .addCommand(specimen2ScoreCommand)
            .addWaitCommand(.2)
            .addCommand(this::rearrangeSpecimen)
            .addWaitCommand(.5)
            .addCommand(closeOuttake)
            .addWaitCommand(.1)
            .addCommand(this::highChamber)
            .build();
    CommandSequence scoreSpecimen3 = new CommandSequence()
            .addCommand(this::scoreSpecimen)
            .addWaitCommand(.4)
            .addCommand(this::releaseSpecimen)
            .build();

    AutoCommandMachine autoMachine = new AutoCommandMachine()
            .addCommandSequence(movepreload)
            .addCommandSequence(scoreSpecimen1)
            .addCommandSequence(pushSample1)
            .addCommandSequence(alignSample2)
            .addCommandSequence(pushSample2)
            .addCommandSequence(alignSpecimen2)
            .addCommandSequence(grabSpecimen2)
            .addCommandSequence(scoreSpecimen2)
            .addCommandSequence(grabSpecimen3)
            .addCommandSequence(scoreSpecimen3)
            .build();

    @Override
    public void init() {
        diffy.init(hardwareMap);
        slides.init(hardwareMap);
        outtake.init(hardwareMap);
        base = new Follower(hardwareMap);
        base.setPose(START_POSE);

        preload = new Path(new BezierLine(
                new Point(7, 54.500, Point.CARTESIAN),
                new Point(38, 63, Point.CARTESIAN)));
        preload.setConstantHeadingInterpolation(Math.toRadians(180));
        sample1Align = new Path(new BezierCurve(
                new Point(38, 63, Point.CARTESIAN),
                new Point(24.590, 51.934, Point.CARTESIAN),
                new Point(32, 35.5, Point.CARTESIAN)));
        sample1Align.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45));
        sample1Push = new Path(new BezierLine(
                new Point(32, 35.5, Point.CARTESIAN),
                new Point(23, 36.5, Point.CARTESIAN)));
        sample1Push.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-135));
        sample2Align = new Path(new BezierLine(
                new Point(23, 36.5, Point.CARTESIAN),
                new Point(33, 26, Point.CARTESIAN)));
        sample2Align.setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-45));
        sample2Push = new Path(new BezierLine(
                new Point(33, 26, Point.CARTESIAN),
                new Point(23, 27, Point.CARTESIAN)));
        sample2Push.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-140));
        specimen1Align = new Path(new BezierLine(
                new Point(23, 27, Point.CARTESIAN),
                new Point(12.5, 42.5, Point.CARTESIAN)));
        specimen1Align.setLinearHeadingInterpolation(Math.toRadians(-140), Math.toRadians(-90));
        specimen1Score = new Path(new BezierCurve(
                new Point(12.5, 42.5, Point.CARTESIAN),
                new Point(16.721, 61.967, Point.CARTESIAN),
                new Point(38, 64, Point.CARTESIAN)));
        specimen1Score.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180));
        specimen1Score.setReversed(true);
        specimen2Align = new Path(new BezierCurve(
                new Point(38, 64, Point.CARTESIAN),
                new Point(15.148, 63.148, Point.CARTESIAN),
                new Point(12.5, 42.5, Point.CARTESIAN)));
        specimen2Align.setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90));
        specimen2Score = new Path(new BezierCurve(
                new Point(12.5, 42.5, Point.CARTESIAN),
                new Point(26.754, 66.885, Point.CARTESIAN),
                new Point(38, 65.508, Point.CARTESIAN)));
        specimen2Score.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180));

        outtake.initPos();
        diffy.diffyTransfer();
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
        diffy.autoUpdate();
        busy = base.isBusy() || !slides.isDone();
        telemetry.addData("is busy", busy);
        telemetry.addData("follower busy? ", busy);
        telemetry.addData("current command: ", autoMachine.getCurrentCommandIndex());
        telemetry.addData("current error: ", slides.getError());
    }

    public void highChamber(){
        diffy.openClaw();
        diffy.diffyInterposed();
        outtake.specimenPos();
        slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET);
    }

    public void scoreSpecimen(){
        outtake.specimenScorePos();
        slides.lock();
    }

    public void releaseSpecimen(){
        outtake.openClaw();
        slides.intakePos();
        outtake.transferPos();
    }

    public void extendDiffyPush(){
        diffy.diffySearch();
        diffy.openClaw();
        diffy.setDif_ROLL(45);
        diffy.setExtendinator(extendyPos);
    }

    public void extendDiffy(){
        diffy.diffySearch();
        diffy.openClaw();
        diffy.setDif_ROLL(0);
        diffy.setExtendinator(extendyPos);
    }

    public void retractDiffy(){
        diffy.shiftClaw();
        diffy.diffyInterposed();
        diffy.retractExtendy();
    }

    public void rearrangeSpecimen(){
        diffy.closeClaw();
        diffy.diffyTransfer();
        slides.intakePos();
        outtake.transferPos();
        outtake.openClaw();
    }

}