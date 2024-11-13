package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
@TeleOp(name="hangteleop", group="LinearOpMode")
public class hangteleop extends LinearOpMode {
    public DcMotorEx leftslide;

    public DcMotorEx rightslide;

    public Servo ro;

    public Servo lo;
    double rx;
    double speed = 1;
    public DcMotor frmotor;
    public DcMotor brmotor;
    public DcMotor flmotor;
    public DcMotor blmotor;
    public Servo ap;
    public Servo ri;
    public Servo li;

    double denominator;
    public double x;

    public double y;



    @Override
    public void runOpMode() throws InterruptedException{
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        ro = hardwareMap.get(Servo.class,"rightoutake");
        lo = hardwareMap.get(Servo.class, "leftoutake");
        rightslide.setDirection(DcMotor.Direction.REVERSE);
        leftslide.setDirection(DcMotor.Direction.REVERSE);

        ap = hardwareMap.get(Servo.class, "airplane");
        ri = hardwareMap.get(Servo.class,"rightintake");
        li = hardwareMap.get(Servo.class, "leftintake");
        flmotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        blmotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        frmotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        brmotor = hardwareMap.get(DcMotor.class, "right_back_drive");

        waitForStart();
        while (opModeIsActive()) {
            rx = gamepad1.left_stick_y;
            if (gamepad2.dpad_up) {
                speed = 1;
            }
            if (gamepad2.dpad_right) {
                speed = 0.75;
            }
            if (gamepad2.dpad_down) {
                speed = 0.50;
            }
            if (gamepad2.dpad_left) {
                speed = 0.25;
            }

            if(gamepad1.b) {
                lo.setPosition(0.5);
                ro.setPosition(0.5);
            }
            if(gamepad1.a){
                lo.setPosition(0);
                ro.setPosition(1);
            }
            rightslide.setPower(gamepad2.left_stick_y * speed);
            leftslide.setPower(gamepad2.left_stick_y * speed);
            rx = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            y = -gamepad1.right_stick_x;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            flmotor.setPower((y - x + rx) / denominator * speed);
            blmotor.setPower((y + x + rx) / denominator * speed);
            frmotor.setPower((y - x - rx) / -denominator * speed);
            brmotor.setPower((y + x - rx) / denominator * speed);

        }

    }


}
