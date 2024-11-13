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
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Disabled
public class pidwithjoysticks extends LinearOpMode {
    private DcMotorEx leftslide;
    private DcMotorEx rightslide;

    private PIDController controller;
    public static double p= 0.015, i=0, d=0;
    public static double f = 0.01;
    private final double ticks_in_degree = 537.7 / 180.0;
    public static int target = 0;
    public double speed =0.75;

    private Servo mo;
    private Servo umo;
    private Servo ro;
    private Servo lo;
    private Servo cl;






    @Override
    public void runOpMode() {
        // Put all of your initialization here.
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ro = hardwareMap.get(Servo.class, "rightoutake");
        lo = hardwareMap.get(Servo.class, "leftoutake");
        mo = hardwareMap.get(Servo.class, "midoutake");
        cl = hardwareMap.get(Servo.class, "claw");
        umo = hardwareMap.get(Servo.class, "uppermidoutake");

        waitForStart();

        int pose = 0;


        // We will use this variable to determine if we want the PIDF to run.
        boolean usePIDF = false;

        Gamepad lastGamepad1 = new Gamepad();
        Gamepad lastGamepad2 = new Gamepad();

        while (opModeIsActive()) {


            // This is a rising-edge detector that runs if and only if "a" was pressed this loop.
            if ((gamepad2.a && !lastGamepad2.a)||(gamepad2.x && !lastGamepad2.x)) {
                usePIDF = true;
            }

            if (gamepad2.x && !lastGamepad2.x) {
                usePIDF = true;
            }


            if (gamepad2.right_trigger > 0) {
                leftslide.setPower(gamepad2.right_trigger * speed);
                rightslide.setPower(gamepad2.right_trigger * speed);

                usePIDF = false;
                // If we get any sort of manual input, turn PIDF off.
            } else if (gamepad2.left_trigger > 0) {
                leftslide.setPower(gamepad2.left_trigger*-speed);
                rightslide.setPower(gamepad2.left_trigger*-speed);

                usePIDF = false;

            } else if (usePIDF && gamepad2.a) {
                // Sets the slide motor power according to the PIDF output.
                /*
            Calculates PID based only on one encoder.
            This can also be the average position of the two linear slides, but we haven't noticed much difference
            */

                pose = 1;
                sleep(100);
                mo.setPosition(0.0);
                ro.setPosition(0.18);
                lo.setPosition(0.82);
                sleep(300);
                cl.setPosition(0.47);
                sleep(300);
                ro.setPosition(0.03);
                lo.setPosition(0.97); // lowest positioj
                sleep(100);
                umo.setPosition(0.0);



            } if (usePIDF && gamepad2.x) {
                /*
            Calculates PID based only on one encoder.
            This can also be the average position of the two linear slides, but we haven't noticed much difference
            */
                pose = 2;

            }
            telemetry.addData("pos ", leftslide.getCurrentPosition());
            telemetry.addData("pos1 ", rightslide.getCurrentPosition());
            telemetry.addData("a ",pose);
            telemetry.addData("x", pose);
            telemetry.update();

            switch (pose){
                case 1:
                    slidePID(100);
                    break;
                case 2:
                    slidePID(0);
                    break;

            }
        }
    }
    public void slidePID(int target){

        controller.setPID(p,i,d);
        int armPos = leftslide.getCurrentPosition();
        int armPos1 = rightslide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;

        leftslide.setPower(power*speed);
        rightslide.setPower(power*speed);

        telemetry.addData("pos ", armPos);
        telemetry.addData("pos1 ", armPos1);
        telemetry.addData("target ",target);
        telemetry.update();

    }
    // OpMode code goes here
}
