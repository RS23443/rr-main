/*
*  what to do:
* 1st create a pidf class for the robot using the kookybotz video, this will be done on a different code
* 2nd use these values and import it here to create working code for your own slides while being able to use joysticks, remember when
* using joysticks you must click the float value not boolean and make sure its y-direction
* only need on pidf controller because there isnt much difference with tow also ue the ftc cookbook for more info and syncing-> kookybotz is only for tuning
* */




package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp
@Disabled
public class pdifwithjoysticks extends LinearOpMode {
    private DcMotorEx leftslide;
    private DcMotorEx rightslide;
    public double speed = 1;

    // This line creates a PIDF controller named examplePIDF that has coefficients of:
    // kP = 0
    // kI = 0
    // kD = 0
    // kF = 0
    private PIDFController examplePIDF = new PIDFController(0.0178,0,0,0.01);


    @Override
    public void runOpMode() {
        // Put all of your initialization here.
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        rightslide.setDirection(DcMotorEx.Direction.REVERSE);
        leftslide.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        int pose = 0;


        // We will use this variable to determine if we want the PIDF to run.
        boolean usePIDF = false;

        Gamepad lastGamepad1 = new Gamepad();
        Gamepad lastGamepad2 = new Gamepad();

        while (opModeIsActive()) {

            // This is a rising-edge detector that runs if and only if "a" was pressed this loop.
            if (gamepad2.a && !lastGamepad2.a) {
                usePIDF = true;
            }

            if (gamepad2.x && !lastGamepad2.x){
                usePIDF = true;
            }
            if (gamepad2.left_trigger > 0) {
                leftslide.setPower(-gamepad2.left_trigger*speed);
                rightslide.setPower(-gamepad2.left_trigger* speed);

                // If we get any sort of manual input, turn PIDF off.
                usePIDF = false;
            }

            if (gamepad2.right_trigger > 0) {
                leftslide.setPower(gamepad2.right_trigger*speed);
                rightslide.setPower(gamepad2.right_trigger* speed);

                // If we get any sort of manual input, turn PIDF off.
                usePIDF = false;
            } else if (usePIDF && gamepad2.a && !lastGamepad2.a) {
                // Sets the slide motor power according to the PIDF output.
                /*
            Calculates PID based only on one encoder.
            This can also be the average position of the two linear slides, but we haven't noticed much difference
            */
                //rightslide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                //leftslide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                pose =1;
                double power = examplePIDF.calculate(leftslide.getCurrentPosition(), pose);

                // see how both motors are getting the same power
                leftslide.setPower(-power);
                rightslide.setPower(-power);



                // If we get any sort of manual input, turn PIDF off

            } else if (usePIDF && gamepad2.x && !lastGamepad2.x) {
                /*
            Calculates PID based only on one encoder.
            This can also be the average position of the two linear slides, but we haven't noticed much difference
            */
                pose = 2;
                double power = examplePIDF.calculate(leftslide.getCurrentPosition(),pose);

                // see how both motors are getting the same power
                leftslide.setPower(power);
                rightslide.setPower(power);

            }
            telemetry.addData("pos ", leftslide.getCurrentPosition());
            telemetry.addData("pos1 ", rightslide.getCurrentPosition());
            telemetry.addData("a ",pose);
            telemetry.addData("x", pose);
            telemetry.update();

            switch (pose){
                case 1:
                    pose = 100;
                    break;
                case 2:
                    pose =0;
                    break;

            }
        }
    }
        // OpMode code goes here
}



