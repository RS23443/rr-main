package org.firstinspires.ftc.teamcode.teleop;

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


@Config
@Disabled
@TeleOp(name="cri drivebase", group="LinearOpMode")
public class cri extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.015, i = 0, d = 0;
    public static double f = 0.01;
    private final double ticks_in_degree = 537.7 / 180.0;
    public static int target = 0;



    private PIDController controller1;
    public DcMotorEx leftslide;

    public DcMotorEx rightslide;

    public Servo ro;

    public Servo lo;
    double rx;
    double speed = 1;
    double speed3 = 0.75;
    public DcMotor frmotor;
    public DcMotor brmotor;
    public DcMotor flmotor;
    public DcMotor blmotor;
    public DcMotor intake;
    public Servo umo;
    public Servo cl;
    public Servo mo;
    public Servo ap;
    public Servo ri;
    public Servo li;

    double denominator;
    public double x;

    public double y;


    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        ro = hardwareMap.get(Servo.class, "rightoutake");
        lo = hardwareMap.get(Servo.class, "leftoutake");
        mo = hardwareMap.get(Servo.class, "midoutake");
        ap = hardwareMap.get(Servo.class, "airplane");
        ri = hardwareMap.get(Servo.class, "rightintake");
        li = hardwareMap.get(Servo.class, "leftintake");
        cl = hardwareMap.get(Servo.class, "claw");
        umo = hardwareMap.get(Servo.class, "uppermidoutake");
        flmotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        blmotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        frmotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        brmotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        brmotor.setDirection(DcMotor.Direction.REVERSE);
        //rightslide.setDirection(DcMotor.Direction.REVERSE);
        //leftslide.setDirection(DcMotor.Direction.REVERSE);
        ap.setPosition(0.1);



        waitForStart();

        int pose =0;


        while (opModeIsActive()) {


            if (gamepad2.left_bumper){

                sleep(100);
                mo.setPosition(0.0);
                ro.setPosition(0.18);
                lo.setPosition(0.82);
                sleep(200);
                cl.setPosition(0.43);
                sleep(200);
                ro.setPosition(0.03);
                lo.setPosition(0.97); // lowest positioj
                sleep(100);
                umo.setPosition(0.0);
            }

            if (gamepad2.x){
                cl.setPosition(0.0);
                sleep(400);
                umo.setPosition(0.0);
                sleep(7000);
                ro.setPosition(0.14);
                lo.setPosition(0.86);
                mo.setPosition(0.03); // was 0.01 did not update this part yet
                sleep(100);


            }
            if(gamepad2.y){
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.4);
                lo.setPosition(0.6);
                sleep(200);
                umo.setPosition(0.6);
                mo.setPosition(0.0);
                sleep(100);

            }
            if (gamepad2.a) {
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.4);
                lo.setPosition(0.6);
                sleep(200);
                umo.setPosition(0.6);
                mo.setPosition(0.0);
                sleep(100);

            }
            if (gamepad2.b) {
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.42);
                lo.setPosition(0.58);
                sleep(200);
                umo.setPosition(0.6);
                mo.setPosition(0.0);
                sleep(100);

            }
            if(gamepad2.right_bumper){
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.5);
                lo.setPosition(0.5);
                sleep(200);
                umo.setPosition(0.6);
                mo.setPosition(0.0);
            }
            if (gamepad1.y){
                ap.setPosition(0.6);
            }
            //one stack
            if (gamepad2.dpad_down){
                li.setPosition(0.28);
                ri.setPosition(0.72);
            }


            //fivestack
            //left is side w plane
            //right is the motor side (for intake)
            //commented out is the old one
            if (gamepad2.dpad_up){
                li.setPosition(0.05);
                ri.setPosition(0.95);

            }
            if(gamepad2.dpad_right){
                mo.setPosition(0.34);
            }
            if (gamepad2.dpad_left){
                mo.setPosition(0.1);
            }


            if (gamepad2.right_bumper) {
                //resets the ro, lo, mo, umo, cl
                // starting pose
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.4);
                lo.setPosition(0.6);
                sleep(200);
                umo.setPosition(0.6);
                mo.setPosition(0.0);
            }

            if (gamepad2.dpad_down) {
                ri.setPosition(0.72);
                li.setPosition(0.28);
            }

            if (gamepad2.dpad_up) {
                li.setPosition(0.0);
                ri.setPosition(1.0);
            }
            if (gamepad2.dpad_left) {
                mo.setPosition(0.34);
            }
            if (gamepad2.dpad_right){
                mo.setPosition(0.34);
            }

// gamepad 1's only had one servo
            if (gamepad1.left_bumper) {
                ap.setPosition(0.6);
            }

            if (gamepad1.a) {
                cl.setPosition(0.0);
            }

            double speed2 = gamepad2.left_stick_y;
            intake.setPower(-speed2);
            rx = gamepad1.left_stick_x * 1.1;
            x = gamepad1.right_stick_x ;
            y = -gamepad1.left_stick_y;

            if (gamepad1.dpad_up) {
                speed = 1;
            }
            if (gamepad1.dpad_right) {
                speed = 0.75;
            }
            if (gamepad1.dpad_down) {
                speed = 0.50;
            }
            if (gamepad1.dpad_left) {
                speed = 0.25;
            }

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            flmotor.setPower((y  + rx + x) / denominator * speed);
            blmotor.setPower((y  - rx + x) / denominator * speed);
            frmotor.setPower((y - rx - x) / denominator * speed);
            brmotor.setPower((y  + rx - x) / denominator * speed);
            rightslide.setPower(gamepad2.right_stick_y * speed3);
            leftslide.setPower(gamepad2.right_stick_y * speed3);


        }
        switch (pose){
            case 1:
                slidePID(100);
                break;
            case 2:
                slidePID(0);
                break;
            case 3:
                slidePID(1500);
                break;
            case 4:
                slidePID(2250);
                break;
            case 5:
                slidePID(2600);
                break;


        }

    }
    public void slidePID(int target){

        controller.setPID(p,i,d);
        int armPos = leftslide.getCurrentPosition();
        int armPos1 = rightslide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;

        leftslide.setPower(power);
        rightslide.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("pos1 ", armPos1);
        telemetry.addData("target ",target);
        telemetry.update();

    }
}