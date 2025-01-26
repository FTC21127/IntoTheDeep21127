
package org.firstinspires.ftc.teamcode.opMode.minibot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Minibot Drive by Time", group="Timed Auto")
public class minibotAuto extends LinearOpMode {
    // you'll get an error after typing this line, that's because you havent overrided and placed code (line 19-20)
// initialize four motors
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    // init elapsed time specially if auton is time based
    private ElapsedTime runtime = new ElapsedTime();
    // set a forward speed and turn speed
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode() {
        // initialize the aforementioned variables
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // since this is a holonomic drive, reverse the left/ right side motors

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //now your motors are initialized you're ready to play
        telemetry.addData("Status","Ready to run");
        telemetry.update();

        //wait for play
        waitForStart();

        // first drive forward for 3 seconds
        backLeft.setPower(FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        frontLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(FORWARD_SPEED);


    }
}
