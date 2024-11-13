package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="main", group="LinearOpMode")
public class teleop extends LinearOpMode {
    private PIDController controller;
    private PIDController controller1;
    public static double p= 0.03, i=0, d=0.0006;
    public static double p1 = 0.03,i1 = 0,d1 = 0.0006;
    public static double f = 0.01;
    public static double f1 = 0.01;
    public static int target = 0;
    private final double ticks_in_degree = 537.7 / 180.0;
    public DcMotorEx frmotor; // control hub 2
    public DcMotorEx brmotor; // control hub 3
    public DcMotorEx flmotor; // control hub 0
    public DcMotorEx blmotor; //control hub 1
    private DcMotorEx leftslide; //exapnsion hub 0
    private DcMotorEx rightslide; //expansion hub 1

    public CRServo intake; // control hub servo 0
    public Servo lif; // control hub servo 1
    public Servo lext; // control hub servo 2
    public Servo lof; // control hub servo 3
    public Servo finger; // control hub servo 4
    public Servo rif; //expansion hub servo 0
    public Servo rext; // expansion hub servo 1
    public Servo rof; // expansion hub servo 2

    public static int pos = 0;
    public double x;
    public double rx;
    public double y;
    public double ry;

    public double denominator;
    public double speed = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller1 = new PIDController(p1,i1,d1);
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        intake = hardwareMap.get(CRServo.class, "intake");
        lif = hardwareMap.get(Servo.class,"left_intake_flip");
        lext = hardwareMap.get(Servo.class, "left_extension");
        lof = hardwareMap.get(Servo.class, "left_outtake_flip");
        finger = hardwareMap.get(Servo.class, "finger");
        rif = hardwareMap.get(Servo.class, "right_intake_flip");
        rext = hardwareMap.get(Servo.class,"right_extension");
        rof = hardwareMap.get(Servo.class, "right_outtake-flip");
        frmotor = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        flmotor = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        brmotor = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        blmotor = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        //rightslide.setDirection(DcMotor.Direction.REVERSE);
        //brmotor.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //ap.setPosition(0.1);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.left_bumper){
                //mo.setPosition(0);
                //li.setPosition(0.3);
                //ri.setPosition(0.7);
            }
            if(gamepad2.y){
                //mo.setPosition(0.5);
            }
            if (gamepad2.a) {
                //pos = 1;
                sleep(200);
                //lo.setPosition(0.33);
                //ro.setPosition(0.67);
            }
            if (gamepad2.b) {
                //pos = 2;
                //lo.setPosition(0.33);
                //ro.setPosition(0.67);
            }
            if(gamepad2.right_bumper){

                //lo.setPosition(0.02);
                //ro.setPosition(0.98);
                sleep(200);
                //pos = 3;
            }
            if (gamepad1.left_bumper){
                //ap.setPosition(0.6);
            }
            //one stack
            if (gamepad2.dpad_down){
                //li.setPosition(0.28);
                //ri.setPosition(0.72);
            }
            //fivestack
            //left is side w plane
            //right is the motor side (for intake)
            //commented out is the old one
            if (gamepad2.dpad_up){
                //li.setPosition(0.0);
                //ri.setPosition(1.0);

            }
            if(gamepad2.dpad_right){
                   //ri.setPosition(0.43);
                   //li.setPosition(0.59);
                //ri.setPosition(0.47);
                //li.setPosition(0.71);
            }
            double speed2 = gamepad2.right_stick_y;
            intake.setPower(1.5*-speed2);
            rx = gamepad1.right_stick_x;
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y ;
            ry = gamepad1.right_stick_y;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            flmotor.setPower((y+x) / denominator * speed);
            blmotor.setPower((y-x) / denominator * speed);
            frmotor.setPower((ry-rx) / denominator * speed);
            brmotor.setPower((ry+rx) / denominator * speed);
            rightslide.setPower(gamepad2.left_stick_y * speed);
            leftslide.setPower(gamepad2.left_stick_y * speed);






            /*switch (pos){
                case 1:
                    slidePID(100);
                    break;
                case 2:
                    slidePID(0);
                    break;
                case 3:
                    slidePID(1000);
                    break;
                case 4:
                    slidePID(2000);
                    break;
                case 5:
                    slidePID(3000);
                    break;
            }*/
        }
    }





/*
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
*/
}
