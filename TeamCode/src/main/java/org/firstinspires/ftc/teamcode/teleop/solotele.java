package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Disabled
@TeleOp(name="solotele", group="LinearOpMode")
public class solotele extends LinearOpMode {
    //private PIDController controller;
    //private PIDController controller1;
    //public static double p= 0.03, i=0, d=0.0006;
    //public static double p1 = 0.03,i1 = 0,d1 = 0.0006;
    //public static double f = 0.01;
    //public static double f1 = 0.01;
    //public static int target = 0;
    //private final double ticks_in_degree = 537.7 / 180.0;
    public DcMotor frmotor;
    public DcMotor brmotor;
    public DcMotor flmotor;
    public DcMotor blmotor;
    //private DcMotorEx leftslide;
    //private DcMotorEx rightslide;
    public DcMotor intake;
    public Servo ro;
    public Servo lo;
    public Servo mo;
    public Servo umo;
    public Servo cl;
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
        //controller = new PIDController(p, i, d);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //controller1 = new PIDController(p1,i1,d1);
        //leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        //rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        //intake = hardwareMap.get(DcMotorEx.class, "intake");
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
        brmotor.setDirection(DcMotor.Direction.REVERSE);

        //rightslide.setDirection(DcMotor.Direction.REVERSE);
        //ap.setPosition(0.1);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                mo.setPosition(0.32);
                //ap.setPosition(0);
                //li.setPosition(0.3);
                //ri.setPosition(0.7);
            }
            if (gamepad1.y) {
                li.setPosition(0.28);
                ri.setPosition(0.72);
            }
            if (gamepad1.a) { // endpoint
                ro.setPosition(0.0); // have to tun clockwise to get desired placement of arm
                lo.setPosition(1.0);
            }
            if (gamepad1.b) { // strt point
                ro.setPosition(0.5);// need to chnage now
                lo.setPosition(0.5); // need to chnage no
            }

            if (gamepad1.x) {
                mo.setPosition(0.5);
            }

            if (gamepad1.right_bumper) {
                umo.setPosition(0.02);
            }

            if (gamepad1.dpad_down) {
                cl.setPosition(0.0);
            }

            if (gamepad1.dpad_up) {
                cl.setPosition(0.65);
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.

            x = x*1.1;



            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            flmotor.setPower(frontLeftPower);
            blmotor.setPower(backLeftPower);
            frmotor.setPower(frontRightPower);
            brmotor.setPower(backRightPower);
        }

    }
}
