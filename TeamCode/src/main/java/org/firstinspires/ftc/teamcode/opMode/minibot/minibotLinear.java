package org.firstinspires.ftc.teamcode.opMode.minibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Minibot TeleOp", group="Linear OpMode")
// add name and group for teleop

public class minibotLinear extends LinearOpMode{
    // declare each DcMotor and ElapsedTime
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public void runOpMode(){
        // initialize each motor by using hardware map to get the DcMotor class
        // name the device the same as what you declared it as

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //now use setDirection to make the left side or right side motors reverse
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // use telemetry.addData, add a caption which is the status and the value
        // then update the telemetry
        telemetry.addData("Status","Initialized");
        telemetry.update();
        //now wait for start and reset the runtime
        waitForStart();
        runtime.reset();
        //run this till the end of the match (while opmode is active)
        // declare the double max
        while(opModeIsActive()){
            double max;
            // declare axial, lateral, and yaw on respective gamepads
            // make sure axial and lateral are on same gamepad
            // axial value is negative

            //get trigger value on right
            double triggervalue =gamepad1.right_trigger;
            // get trigger value on left
            double triggervaluel = gamepad1.left_trigger;

            double axial = -gamepad1.left_stick_y * (1- triggervalue);
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x * (1- triggervaluel);

            //left front and right front are opposites,
            //left back and right back are opposites

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower),(Math.abs(frontRightPower)));
            max = Math.max(max,Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            // max=  abs front left and right
            // max= max, and abs leftback
            // max = max, and abs rightback

            if (max >1.0){
               frontLeftPower /= max;
               frontRightPower /=max;
               backRightPower /=max;
               backLeftPower /= max;

            }
            //if max >1.0, all /= max.

            // set drive power to motorpower
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Status", "Runtime" + runtime.toString());
            //telemetry, add status and runtime + runetime to string
            //that's it.



        }





    }









}
