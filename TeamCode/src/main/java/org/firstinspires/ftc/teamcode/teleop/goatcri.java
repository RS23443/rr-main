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
@TeleOp(name="phantasmal yapper", group="LinearOpMode")
public class goatcri extends LinearOpMode {
    private PIDController controller;
    private PIDController controller1;
    public static double p= 0.0178, i=0, d=0;
    public static double p1 = 0.0178,i1 = 0,d1 = 0;
    public static double f = 0.01;
    public static double f1 = 0.01;
    public static int target = 0;
    private final double ticks_in_degree = 537.7 / 180.0;
    public DcMotor frmotor;
    public DcMotor brmotor;
    public DcMotor flmotor;
    public DcMotor blmotor;
    private DcMotorEx leftslide;
    private DcMotorEx rightslide;
    public DcMotorEx intake;
    public Servo ro;
    public Servo lo;
    public Servo mo;
    public Servo ap;
    public Servo ri;
    public Servo li;
    public Servo umo;
    private Servo cl;
    public static int pos = 0;
    public double x;
    public double rx;
    public double y;

    public double denominator;
    public double speed = 1;
    public double speed3 = 0.75;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        cl = hardwareMap.get(Servo.class, "claw");
        umo = hardwareMap.get(Servo.class, "uppermidoutake");
        flmotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        blmotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        frmotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        brmotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        //frmotor.setDirection(DcMotor.Direction.REVERSE);
        rightslide.setDirection(DcMotor.Direction.REVERSE);
        leftslide.setDirection(DcMotor.Direction.REVERSE);
        ap.setPosition(0.1);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.left_bumper){
                pos =1;
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
                sleep(500);
                ro.setPosition(0.11);
                lo.setPosition(0.89);
                mo.setPosition(0.03); // was 0.01 did not update this part yet
                sleep(100);
                pos = 2;

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
                pos = 5;
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
                pos = 3;
            }
            if (gamepad2.b) {
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.4);
                lo.setPosition(0.6);
                sleep(200);
                umo.setPosition(0.6);
                mo.setPosition(0.0);
                sleep(100);
                pos = 4;
            }
            if(gamepad2.right_bumper){
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.42);
                lo.setPosition(0.58);
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

            if (gamepad1.a){
                cl.setPosition(0.0);
            }

            if (gamepad1.x){
                cl.setPosition(0.6);
                sleep(200);
                ro.setPosition(0.4);
                lo.setPosition(0.6);
                sleep(200);
                umo.setPosition(0.0);
                mo.setPosition(0.0);
                sleep(100);
                pos = 6;

            }
            //TEST SETPOWER
            if (gamepad1.b){
                flmotor.setPower(0.1);
                blmotor.setPower(-0.1);
                frmotor.setPower(-0.1);
                brmotor.setPower(0.1);
                sleep(3000);
                flmotor.setPower(0);
                blmotor.setPower(0);
                frmotor.setPower(0);
                brmotor.setPower(0);

            }
            if (gamepad1.left_trigger > 0) {
                rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftslide.setPower(-gamepad2.left_trigger*speed);
                rightslide.setPower(-gamepad2.left_trigger* speed);

            }

            if (gamepad1.right_trigger > 0) {
                rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftslide.setPower(gamepad2.right_trigger*speed);
                rightslide.setPower(gamepad2.right_trigger* speed);

            }
            double speed2 = gamepad2.left_stick_y;
            intake.setPower(-speed2);
            rx = gamepad1.left_stick_x * 1.1;
            x = gamepad1.right_stick_x ;
            y = -gamepad1.left_stick_y;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

            flmotor.setPower((y  + rx + x) / denominator * speed);
            blmotor.setPower((y  + rx - x) / denominator * speed);
            frmotor.setPower((y - rx - x) / denominator * speed);
            brmotor.setPower((y  - rx + x) / denominator * speed);
            //rightslide.setPower(gamepad1.left_stick_y * speed);
            //leftslide.setPower(gamepad1.left_stick_y * speed);






            switch (pos){
                case 1:
                    slidePID(130);
                    break;
                case 2:
                    slidePID(10);
                    break;
                case 3:
                    slidePID(900);
                    break;
                case 4:
                    slidePID(1500);
                    break;
                case 5:
                    slidePID(2000);
                    break;

                case 6:
                    slidePID(500);
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
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f* speed3;
        double ff1 = Math.cos(Math.toRadians(target /ticks_in_degree)) * f1 * speed3;
        double power = (pid + ff)*speed3;
        double power1 = (pid1 + ff1)*speed3;

        leftslide.setPower(power1);
        rightslide.setPower(power1);

        telemetry.addData("pos ", armPos);
        telemetry.addData("pos1 ", armPos1);
        telemetry.addData("target ",target);
        telemetry.update();

    }

}
