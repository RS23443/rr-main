package org.firstinspires.ftc.TeamCode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="main", group="LinearOpMode")
public class teleop extends LinearOpMode {
    private PIDController controller;
    private PIDController controller1;
    public static double p=0.03, i=0, d=0.0006;
    public static double p1 = 0.03,i1 = 0,d1 = 0.0006;
    public static double f = 0.01;
    public static double f1 = 0.01;
    public static int target = 0;
    private final double ticks_in_degree = 537.7 / 180.0;
    public DcMotor frmotor;
    public DcMotor brmotor;
    public DcMotor flmotor;
    public DcMotor blmotor;
    public DcMotorEx leftslide;
    public DcMotorEx rightslide;
    public DcMotorEx intake;
    public Servo ro;
    public Servo lo;
    public Servo mo;
    public Servo ap;
    public Servo ri;
    public Servo li;
    public static int pos = 0;
    public double x;
    public double rx;
    public double y;

    public double denominator;
    public double speed = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        controller1 = new PIDController(p1,i1,d1);
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        ro = hardwareMap.get(Servo.class,"rightoutake");
        lo = hardwareMap.get(Servo.class, "leftoutake");
        mo = hardwareMap.get(Servo.class, "midoutake");
        ap = hardwareMap.get(Servo.class, "airplane");
        ri = hardwareMap.get(Servo.class,"rightintake");
        li = hardwareMap.get(Servo.class, "leftintake");
        flmotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        blmotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        frmotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        brmotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightslide.setDirection(DcMotor.Direction.REVERSE);
        ap.setPosition(0.1);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.left_bumper){
                mo.setPosition(0);
                li.setPosition(0.3);
                ri.setPosition(0.7);
            }
            if(gamepad2.y){
                mo.setPosition(0.5);
            }
            if (gamepad2.a) {
                pos = 1;
                sleep(200);
                lo.setPosition(0.33);
                ro.setPosition(0.67);
            }
            if (gamepad2.b) {
                pos = 2;
                lo.setPosition(0.33);
                ro.setPosition(0.67);
            }
            if(gamepad2.right_bumper){

                lo.setPosition(0);
                ro.setPosition(1);
                sleep(200);
                pos = 3;
            }
            if (gamepad1.left_bumper){
                ap.setPosition(0.6);
            }
            if (gamepad2.dpad_down){
                ri.setPosition(0.3);
                li.setPosition(0.7);
            }
            if (gamepad2.dpad_up){
                ri.setPosition(0.41);
                li.setPosition(0.59);
            }
            if(gamepad2.dpad_right){
                ri.setPosition(0.46);
                li.setPosition(0.56);
            }
            double speed2 = gamepad2.right_stick_y;
            intake.setPower(-speed2*0.8);
            rx = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            y = -gamepad1.right_stick_x;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            flmotor.setPower((y - x + rx) / denominator * speed);
            blmotor.setPower((y + x + rx) / denominator * speed);
            frmotor.setPower((y - x - rx) / denominator * speed);
            brmotor.setPower((y + x - rx) / denominator * speed);
            //rightslide.setPower(gamepad1.left_stick_y * speed);
            //leftslide.setPower(gamepad1.left_stick_y * speed);






            switch (pos){
                case 1:
                    slidePID(1500);
                    break;
                case 2:
                    slidePID(2600);
                    break;
                case 3:
                    slidePID(-5);
                    break;
            }
        }
    }






    public void slidePID(int target){

        controller.setPID(p,i,d);
        controller1.setPID(p1,i1,d1);
        int armPos = leftslide.getCurrentPosition();
        int armPos1 = rightslide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double pid1 = controller1.calculate(armPos1, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double ff1 = Math.cos(Math.toRadians(target /ticks_in_degree)) * f1;
        double power = pid + ff;
        double power1 = pid1 + ff1;

        leftslide.setPower(power);
        rightslide.setPower(power1);

        telemetry.addData("pos ", armPos);
        telemetry.addData("pos1 ", armPos1);
        telemetry.addData("target ",target);
        telemetry.update();

    }

}
