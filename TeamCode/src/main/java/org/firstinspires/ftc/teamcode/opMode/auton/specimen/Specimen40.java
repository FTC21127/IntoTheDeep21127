package org.firstinspires.ftc.teamcode.opMode.auton.specimen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlides;


@Autonomous(name = "4 + 0'", group = "!specimen", preselectTeleOp = "RedTele")
public class Specimen40 extends OpMode {
    Deposit deposit = new Deposit(this);
    Drivetrain drivetrain = new Drivetrain(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    Intake diffy = new Intake(this);

    private final double[] grabYPos = {};
    private int sampleNumber = 1;

    private final Pose startPose = new Pose(7.48, 54.85, Math.toRadians(180));
    private final Pose preloadPose = new Pose(39, 61.5, Math.toRadians(180));
    private final Pose scorePose = new Pose(39, 66, Math.toRadians(180));
    private final Pose grabPose = new Pose(39, grabYPos[sampleNumber-1], Math.toRadians(180));



    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
