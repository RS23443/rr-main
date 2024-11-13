package org.firstinspires.ftc.teamcode.auto_CenterStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * Adapted from FTC WIRES Core
 */
@SuppressWarnings("RedundantThrows")
@Autonomous(name = "Team 23443 Autonomous 2+0 Truss", group = "00-Autonomous", preselectTeleOp = "TeleOpMode")
public class auto20Truss extends LinearOpMode {

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        RedLeft,
        RedRight,
        BlueLeft,
        BlueRight
    }

    public enum PARK_POSITION {
        Left,
        Right
    }

    public enum TEAM_ELEMENT_POSITION {
        LeftSpike,
        MiddleSpike,
        RightSpike
    }

    public static START_POSITION startPosition;
    public static PARK_POSITION parkPosition;
    public static TEAM_ELEMENT_POSITION teamElementPosition;

    public SampleMecanumDrive driveTrain;

    OpenCvWebcam webcam;  //Needed for OpenCV detection of the Team Element

    //Define Constants Below (Example)
    //Arm Position for dropping the Purple Pixel
    //int armDropPurpleHeight = 0;
    //Arm Position for dropping the Yellow Pixel

    int armHomeHeight = 0;
    int armDropYellowHeight = 1100;
    public Servo li;
    public Servo ri;
    public Servo ro;
    public Servo lo;
    public  Servo mo;
    public DcMotor leftslide;
    public DcMotor rightslide;
    public DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new SampleMecanumDrive(hardwareMap);
        li = hardwareMap.get(Servo.class, "leftintake");
        ri = hardwareMap.get(Servo.class, "rightintake");
        ro = hardwareMap.get(Servo.class, "rightoutake");
        lo = hardwareMap.get(Servo.class, "rightoutake");
        mo = hardwareMap.get(Servo.class, "midoutake");
        leftslide = hardwareMap.get(DcMotor.class, "leftslide");
        rightslide = hardwareMap.get(DcMotor.class, "rightslide");
        intake = hardwareMap.get(DcMotor.class,"intake");
        rightslide.setDirection((DcMotorSimple.Direction.REVERSE));
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        leftslide.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        rightslide.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        leftslide.setTargetPosition(0);
        rightslide.setTargetPosition(0);
        ri.setPosition(0.3);
        li.setPosition(0.7);
        mo.setPosition(0);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ((DcMotorEx) rightslide).setVelocity(2000);
        ((DcMotorEx) leftslide).setVelocity(2000);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);  //Added to support Bulk Reads

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }  //Added to support Bulk Reads (faster loop execution)

        //Driver inputs to selecting Starting Position of robot from the Gamepad
        selectStartingPosition();

        // Initiate Camera on Init.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //pipeline = new CenterStagePipeline();
        webcam.setPipeline(new CenterStagePipeline());

        //Setup basket for start of Auton
        //Lock the pixels into the basket

        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.clearAll();
            //Incrementing Frame Count confirms that the camera is getting fresh data
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("(Team Name) Team:", "(Team Number)");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Team Element", teamElementPosition);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Shutdown the camera to save CPU cycles
            webcam.closeCameraDevice();
            //Build Autonomous trajectory to be used based on starting position selected
            buildAuto();
            driveTrain.getLocalizer().setPoseEstimate(initPose);
            //Build parking trajectory based on last detected target by vision
            buildParking();
            //run Autonomous trajectory
            runAutoAndParking();
        }
    }

    TrajectorySequence trajectoryParking, trajectory1, trajectory2, trajectory3, trajectory4, trajectory5, trajectory6, trajectory7;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d preParkPose, parkPosePosition, purplePreDropPose, yellowDropPose, purpleDropPose, yellowPreDropPose1, yellowPreDropPose2, finalPurpleDrop;

    //Set all position based on selected starting location and Build Autonomous Trajectory

    //Use this field layout for X and Y
    // -X     BlueR                   BlueL     +X
    //|-------------------------------------------|  +Y
    //|                                           |
    //|       3     1                3     1      | 1
    //|          2                      2         | 2 Backdrop
    //|                                           | 3
    //|                                           |
    //|                   0,0                     |
    //|                                           |
    //|                                           |
    //|                                           | 1
    //|          2                      2         | 2 Backdrop
    //|       1     3                1     3      | 3
    //|                                           |
    //|-------------------------------------------|  -Y
    //         RedL                   RedR
    //
    //Use this compass for angles to match field above
    //              90
    //    180                0
    //              270
    //

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {


            //RED LEFT
            case RedLeft:  //Use this case for Robot starting on Red Left
                initPose = new Pose2d(-34, -63, Math.toRadians(90)); //Starting pose LEFT
                purplePreDropPose = new Pose2d(-34, -45, Math.toRadians(90)); //Pull forward from Start Pose
                yellowPreDropPose2 = new Pose2d(38, -58, Math.toRadians(180));
                switch (teamElementPosition) {
                    case LeftSpike:  //team element drop Position 1 (Left)
                        yellowDropPose = new Pose2d(57, -28, Math.toRadians(180));
                        purpleDropPose = new Pose2d(-41, -39, Math.toRadians(140));
                        yellowPreDropPose1 = new Pose2d(-35, -58, Math.toRadians(180));
                        finalPurpleDrop = new Pose2d(-35, -42, Math.toRadians(180));
                        break;
                    case MiddleSpike:  //team element drop Position 2 (Middle)
                        yellowDropPose = new Pose2d(57, -33, Math.toRadians(180));
                        purpleDropPose = new Pose2d(-34, -33, Math.toRadians(90));
                        yellowPreDropPose1 = new Pose2d(-35, -58, Math.toRadians(180));
                        finalPurpleDrop = new Pose2d(-34,-35,Math.toRadians(90));
                        break;
                    case RightSpike:  //team element drop Position 3 (Right)
                        yellowDropPose = new Pose2d(56, -35, Math.toRadians(180));
                        purpleDropPose = new Pose2d(-26, -36, Math.toRadians(40));
                        yellowPreDropPose1 = new Pose2d(-35, -58, Math.toRadians(180));
                        finalPurpleDrop = new Pose2d(-46,-57, Math.toRadians(90));
                        break;
                }
                preParkPose = new Pose2d(50, -35, Math.toRadians(180)); // Midway to Park Position
                switch (parkPosition) {
                    case Right:  //Park to the Right of the Backdrop
                        parkPosePosition = new Pose2d(50, -60, Math.toRadians(180));
                        break;
                    case Left:  //Park to the Left of the Backdrop
                        parkPosePosition = new Pose2d(55, -10, Math.toRadians(180));
                        break;
                }
                break;


            //REDRIGHT
            case RedRight:  //Use this case for Robot starting on Red Right
                initPose = new Pose2d(13, -60, Math.toRadians(90)); //Starting pose Red Right
                purplePreDropPose = new Pose2d(13, -40, Math.toRadians(90)); //Pull forward from Start Pose
                yellowPreDropPose1 = new Pose2d(40, -45, Math.toRadians(180));
                yellowPreDropPose2 = new Pose2d(40, -30, Math.toRadians(180));
                switch (teamElementPosition) {
                    case LeftSpike:  //team element drop Position 1 (Left)
                        yellowDropPose = new Pose2d(57, -25, Math.toRadians(180));
                        purpleDropPose = new Pose2d(9, -37, Math.toRadians(144));
                        finalPurpleDrop = new Pose2d(15,-40, Math.toRadians(144));
                        break;
                    case MiddleSpike:  //team element drop Position 2 (Middle)
                        yellowDropPose = new Pose2d(57, -34, Math.toRadians(180));
                        purpleDropPose = new Pose2d(13, -29, Math.toRadians(90));
                        finalPurpleDrop = new Pose2d(13,-31, Math.toRadians(90));

                        break;
                    case RightSpike:  //team element drop Position 3 (Right)
                        yellowDropPose = new Pose2d(57, -37, Math.toRadians(180));
                        purpleDropPose = new Pose2d(20, -36, Math.toRadians(31));
                        finalPurpleDrop = new Pose2d(12,-39, Math.toRadians(31));

                        break;
                }
                preParkPose = new Pose2d(45, -40, Math.toRadians(180)); // Midway to Park Position
                switch (parkPosition) {
                    case Right:  //Park to the Right of the Backdrop
                        parkPosePosition = new Pose2d(55, -55, Math.toRadians(180));
                        break;
                    case Left:  //Park to the Left of the Backdrop
                        parkPosePosition = new Pose2d(55, -10, Math.toRadians(180));
                        break;
                }
                break;


            //BLUE LEFT
            case BlueLeft:  //Use this case for Robot starting on Blue Left
                initPose = new Pose2d(13, 63, Math.toRadians(270)); //Starting pose Blue Left
                purplePreDropPose = new Pose2d(14, 43, Math.toRadians(270)); //Pull forward from Start Pose
                yellowPreDropPose1 = new Pose2d(44, 55, Math.toRadians(180));
                yellowPreDropPose2 = new Pose2d(44, 35, Math.toRadians(180));
                switch (teamElementPosition) {
                    case LeftSpike:  //team element drop Position 1 (Left)
                        yellowDropPose = new Pose2d(57, 45, Math.toRadians(180));
                        purpleDropPose = new Pose2d(18, 38, Math.toRadians(340));
                        finalPurpleDrop = new Pose2d(14,40, Math.toRadians(340));
                        break;
                    case MiddleSpike:  //team element drop Position 2 (Middle)
                        yellowDropPose = new Pose2d(58, 37, Math.toRadians(180));
                        purpleDropPose = new Pose2d(15, 33, Math.toRadians(270));
                        finalPurpleDrop = new Pose2d(22,38, Math.toRadians(180));
                        break;
                    case RightSpike:  //team element drop Position 3 (Right)
                        yellowDropPose = new Pose2d(57, 30, Math.toRadians(180));
                        purpleDropPose = new Pose2d(9, 38, Math.toRadians(213));
                        finalPurpleDrop = new Pose2d(11,40, Math.toRadians(213));

                        break;
                }
                preParkPose = new Pose2d(45, 40, Math.toRadians(180)); // Midway to Park Position
                switch (parkPosition) {
                    case Right:  //Park to the Right of the Backdrop
                        parkPosePosition = new Pose2d(55, 15, Math.toRadians(180));
                        break;
                    case Left:  //Park to the Left of the Backdrop
                        parkPosePosition = new Pose2d(55, 55, Math.toRadians(180));
                        break;
                }
                break;


            //BLUE RIGHT
            case BlueRight:  //Use this case for Robot starting on Blue Right
                initPose = new Pose2d(-34, 63, Math.toRadians(270)); //Starting pose LEFT
                purplePreDropPose = new Pose2d(-34, 45, Math.toRadians(270)); //Pull forward from Start Pose
                yellowPreDropPose2 = new Pose2d(38, 55, Math.toRadians(180));
                switch (teamElementPosition) {
                    case LeftSpike:  //team element drop Position 1 (Left)
                        yellowDropPose = new Pose2d(57, 44, Math.toRadians(180));
                        purpleDropPose = new Pose2d(-28, 36, Math.toRadians(321));
                        yellowPreDropPose1 = new Pose2d(-46, 55, Math.toRadians(180));
                        finalPurpleDrop = new Pose2d(-46,50,Math.toRadians(290));
                        break;
                    case MiddleSpike:  //team element drop Position 2 (Middle)
                        yellowDropPose = new Pose2d(57, 37, Math.toRadians(180));
                        purpleDropPose = new Pose2d(-34, 30, Math.toRadians(270));
                        yellowPreDropPose1 = new Pose2d(-34, 55, Math.toRadians(180));
                        finalPurpleDrop = new Pose2d(-34,35,Math.toRadians(270));
                        break;
                    case RightSpike:  //team element drop Position 3 (Right)
                        yellowDropPose = new Pose2d(57, 32, Math.toRadians(180));
                        purpleDropPose = new Pose2d(-43, 39, Math.toRadians(230));
                        yellowPreDropPose1 = new Pose2d(-31, 55, Math.toRadians(180));
                        finalPurpleDrop = new Pose2d(-33,44,Math.toRadians(180));
                        break;
                }
                preParkPose = new Pose2d(35, 30, Math.toRadians(180)); // Midway to Park Position
                switch (parkPosition) {
                    case Right:  //Park to the Right of the Backdrop
                        parkPosePosition = new Pose2d(50, 13, Math.toRadians(180));
                        break;
                    case Left:  //Park to the Left of the Backdrop
                        parkPosePosition = new Pose2d(55, 55, Math.toRadians(180));
                        break;
                }
                break;
        }

        //Section for building trajectories
        //Trajectory 1 is from robot starting position to Purple Pre-Drop
        trajectory1 = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(purplePreDropPose)
                .build();

        //Trajectory 2 is from Purple PreDrop to the actual drop position of the Purple Pixel
        trajectory2 = driveTrain.trajectorySequenceBuilder(trajectory1.end())
                .lineToLinearHeading(purpleDropPose,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .addTemporalMarker(3, () -> {
                    ri.setPosition(0.5);
                    li.setPosition(0.5);
                })
                .build();

        //Trajectory 3 is from Purple Pixel Drop to a Mid Position to clear the Purple Pixel
        trajectory3 = driveTrain.trajectorySequenceBuilder(trajectory2.end())
                .lineToLinearHeading(finalPurpleDrop)
                .build();

        //Trajectory 4 is from Yellow Pixel Mid 1 to Yellow Pixel Mid 2
        trajectory4 = driveTrain.trajectorySequenceBuilder(trajectory3.end())
                .lineToLinearHeading(yellowPreDropPose1)
                .build();

        //Trajectory 5 is from the Yellow Pixel PreDrop pose to the Backdrop dropping pose
        trajectory5 = driveTrain.trajectorySequenceBuilder(trajectory4.end())
                .addTemporalMarker(1, () -> {
                    yellowPixelDropHeight();
                    lo.setPosition(0.3);
                    ro.setPosition(0.7);
                })
                .lineToLinearHeading(yellowPreDropPose2)

                .build();

        //Trajectory 6 is after the Yellow Pixel Drop backing away from the Backdrop
        trajectory6 = driveTrain.trajectorySequenceBuilder(trajectory5.end())
                .lineToLinearHeading(yellowDropPose,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .addTemporalMarker(3, () -> {
                    sleep(100);
                    mo.setPosition(0.5);
                    sleep(500);
                    rightslide.setTargetPosition(1600);
                    leftslide.setTargetPosition(1600);

                })
                .build();

        trajectory7 = driveTrain.trajectorySequenceBuilder(trajectory6.end())
                .addTemporalMarker(3,()->{
                    sleep(500);
                    lo.setPosition(0);
                    ro.setPosition(1);
                    sleep(500);
                    rightslide.setTargetPosition(-10);
                    leftslide.setTargetPosition(-10);
                })
                .lineToLinearHeading(preParkPose,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))

                .build();


    }

    //Build parking trajectory based on target detected by vision
    public void buildParking() {
        //Parking Trajectory
        trajectoryParking = driveTrain.trajectorySequenceBuilder(trajectory7.end())

                .lineToLinearHeading(parkPosePosition)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry.addData("RoboSupport", "(23443))");
        telemetry.addData("---------------------------------------", "");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        //sleep(8000);
        driveTrain.followTrajectorySequence(trajectory1);
        sleep(200);
        driveTrain.followTrajectorySequence(trajectory2);
        sleep(200);
        driveTrain.followTrajectorySequence(trajectory3);
        sleep(200);
        driveTrain.followTrajectorySequence(trajectory4);
        sleep(200);

        driveTrain.followTrajectorySequence(trajectory5);
        sleep(3000);

        driveTrain.followTrajectorySequence(trajectory6);
        sleep(500);
        driveTrain.followTrajectorySequence(trajectory7);
        sleep(200);
        driveTrain.followTrajectorySequence(trajectoryParking);
    }

    //Write a method which will bring the arm to drop height
    //public void purplePixelDropHeight() {
    //armMotor.setTargetPosition(armDropPurpleHeight);
    //}

    public void yellowPixelDropHeight() {
        rightslide.setTargetPosition(armDropYellowHeight);
        leftslide.setTargetPosition(armDropYellowHeight);
    }

    public void mastHeightHome() {
        intake.setPower(1.0);
    }

    //Write a method which is able to drop the Purple Pixel
    //public void dropPurplePixel() {
    //pixelRelease.setPosition(0); //Release the Servo Lock
    //This is the only place you can instert delays, NOT while in a trajectory
    //sleep(1000); //Give the Pixel 1 second to fall out of the basket
    //}

    //public void dropYellowPixel() {
    //sleep(500);
    ///pixelRelease.setPosition(0); //Release Servo Lock
    //sleep(500); //Give the Pixel 0.5 seconds to fall out of the basket
    //armMotor.setTargetPosition(588);
    //sleep(500);
    //}



    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous Mode:", "(23443)");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using Keys on gamepad 1:", "");
            telemetry.addData(" Start BLUE RIGHT Side  ", "(X)");
            telemetry.addData(" Start BLUE LEFT Side  ", "(A)");
            telemetry.addData(" Start RED RIGHT Side  ", "(Y)");
            telemetry.addData(" Start RED LEFT Side ", "(B)");

            if (gamepad1.b) {
                startPosition = START_POSITION.RedLeft;
                break;
            }
            if (gamepad1.x) {
                startPosition = START_POSITION.BlueRight;
                break;
            }
            if (gamepad1.y) {
                startPosition = START_POSITION.RedRight;
                break;
            }
            if (gamepad1.a) {
                startPosition = START_POSITION.BlueLeft;
                break;
            }
            telemetry.update();
        }

        telemetry.clearAll();
        while (!isStopRequested()) {
            telemetry.addData("Select Parking Position", "");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select using Keys on gamepad 2:", "");
            telemetry.addData(" Park on Left of Backdrop  ", "(X)");
            telemetry.addData(" Park on Right of Backdrop ", "(B)");

            if (gamepad2.b) {
                parkPosition = PARK_POSITION.Right;
                break;
            }
            if (gamepad2.x) {
                parkPosition = PARK_POSITION.Left;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }


    //Start logic below for OpenCV Camera System
    //The code below is an adaptation from 'SkystoneDeterminationExample.java' example
    public static class CenterStagePipeline extends OpenCvPipeline {

        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        //The Anchor points are used to define where the region of interest is that you are looking for the team element
        //Note, this is the upper left corner of each box, then the height and width is defined below as 20 pixels square
        //The camera feed is being down sized to 320x240 so don't move the box off the screen or an error will occur (300 is max X, 220 is max y)
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(15, 190);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150, 165);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(290, 190);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        //Working variables
        Mat region1_Cb, region2_Cb, region3_Cb;  //For detecting blue (we will also detect red)
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        //The following function takes the RGB frame and converts it to YCrCb (Cr is the Red Channel, Cb is the Blue Channel)
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            switch (startPosition) {
                case RedLeft:
                    Core.extractChannel(YCrCb, Cb, 1)  //Look for strongest Red box (Index 1 of YCrCb)
                    ;
                    break;
                case RedRight:
                    Core.extractChannel(YCrCb, Cb, 1)  //Look for strongest Red box (Index 1 of YCrCb)
                    ;
                    break;
                case BlueLeft:
                    Core.extractChannel(YCrCb, Cb, 2)  //Look for strongest Blue box (Index 2 of YCrCb)
                    ;
                    break;
                case BlueRight:
                    Core.extractChannel(YCrCb, Cb, 2)  //Look for strongest Blue box (Index 2 of YCrCb)
                    ;
                    break;
            }
        }

        @Override
        public void init(Mat firstFrame) {
            //Call this to insure the Cb object is initialized
            inputToCb(firstFrame);
            //Submats are a persistent reference to a region of the parent buffer.
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            //Get the Cb or Cr channel from the image
            inputToCb(input);
            //Compute the average pixel value of each region
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            //Draw some rectangles to help with troubleshooting and placement from the drive station
            Imgproc.rectangle(
                    input, //Buffer to draw on
                    region1_pointA, //First point which defines the rectangle
                    region1_pointB, //Second point which defines the rectangle
                    BLUE, //Color to draw the rectangle
                    2); //Thickness of the rectangular line

            Imgproc.rectangle(
                    input, //Buffer to draw on
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2);

            Imgproc.rectangle(
                    input, //Buffer to draw on
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2);

            //Find the max of the 3 averages above
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            //Find out which region contains the max
            if (max == avg1)  //Was it the Left Spike?
            {
                teamElementPosition = TEAM_ELEMENT_POSITION.LeftSpike;
                Imgproc.rectangle( //Draw a Solid Green Box if this is the identified position
                        input, region1_pointA, region1_pointB, GREEN, -1);
            }
            if (max == avg2)  //Was it the Middle Spike?
            {
                teamElementPosition = TEAM_ELEMENT_POSITION.MiddleSpike;
                Imgproc.rectangle( //Draw a Solid Green Box if this is the identified position
                        input, region2_pointA, region2_pointB, GREEN, -1);
            }
            if (max == avg3)  //Was it the Right Spike
            {
                teamElementPosition = TEAM_ELEMENT_POSITION.RightSpike;
                Imgproc.rectangle( //Draw a Solid Green Box if this is the identified position
                        input, region3_pointA, region3_pointB, GREEN, -1);
            }
            return input;
        }
    }
}
