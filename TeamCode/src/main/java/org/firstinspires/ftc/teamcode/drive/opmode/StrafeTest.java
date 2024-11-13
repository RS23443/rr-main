package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public DcMotor frmotor;
    public DcMotor brmotor;
    public DcMotor flmotor;
    public DcMotor blmotor;
    public static double DISTANCE = 24; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void strae(){
        flmotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        blmotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        frmotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        brmotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setPower(0.25);
        blmotor.setPower(-0.25);
        frmotor.setPower(-0.25);
        brmotor.setPower(0.25);
        sleep(3000);
        flmotor.setPower(0);
        blmotor.setPower(0);
        frmotor.setPower(0);
        brmotor.setPower(0);
    }
}
