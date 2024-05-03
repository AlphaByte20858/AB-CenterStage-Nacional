package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutoBlueLow;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.ArrayList;
import java.util.List;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Autoninho", group = "LinearOpMode")
public class AutonomusTraining extends LinearOpMode {
        DcMotorEx MET, MEF, MDT, MDF, MBD, MART;
        Servo SGE, SGD;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    public enum mark{
        Left,
        Middle,
        Right
    }
    mark ObjectPos;

    public void runOpMode() {

            MET = hardwareMap.get(DcMotorEx.class, "MET");
            MDT = hardwareMap.get(DcMotorEx.class, "MDT");
            MDF = hardwareMap.get(DcMotorEx.class, "MDF");
            MEF = hardwareMap.get(DcMotorEx.class, "MEF");
            MBD = hardwareMap.get(DcMotorEx.class, "MBD");
            MART = hardwareMap.get(DcMotorEx.class, "Articular");
            SGE = hardwareMap.get(Servo.class, "SGE");
            SGD = hardwareMap.get(Servo.class, "SGD");


            MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MBD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MART.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            MEF.setDirection(DcMotorEx.Direction.REVERSE);
            MDF.setDirection(DcMotorEx.Direction.FORWARD);
            MET.setDirection(DcMotorEx.Direction.REVERSE);
            MDT.setDirection(DcMotorEx.Direction.FORWARD);
            MBD.setDirection(DcMotorEx.Direction.FORWARD);
            MART.setDirection(DcMotorSimple.Direction.FORWARD);
            SGE.setDirection(Servo.Direction.REVERSE);

            MDF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MDT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MEF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MET.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MBD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            MART.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            MBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            TrajectorySequence mid = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-24, 4, Math.toRadians(0)))
                    .strafeTo(new Vector2d(-24, -4))
                    .turn(Math.toRadians(180))
                    .build();

            TrajectorySequence midBD = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-24, 36, Math.toRadians(-90)))
                    .build();

            TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-27,4, Math.toRadians(0)))
                    .strafeRight(26)
                    .turn(Math.toRadians(-100))
                    .build();

            TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-24,6, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .back(5)
                    .turn(Math.toRadians(-15))
                    .build();

            ElapsedTime tempo = new ElapsedTime();
            SGE.setPosition(0);
            SGD.setPosition(0);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();
            if (!isStopRequested())

                if (ObjectPos == mark.Left){
                    drive.followTrajectorySequence(left);
                    tempo.startTime();
                    tempo.reset();
                    while (tempo.seconds()<1){
                        MART.setPower(0.1);
                    }
                    MART.setPower(0);
                    SGE.setPosition(1);
                    tempo.reset();
                    while (tempo.seconds() < 1) {
                        MART.setPower(-0.3);
                    }
                }

                else if (ObjectPos == mark.Middle){
                    drive.followTrajectorySequence(mid);
                    sleep(2000);
                    tempo.reset();
                    while (tempo.seconds() < 1){
                        MART.setPower(0.1);
                    }
                    MART.setPower(0);
                    sleep(1000);
                    SGE.setPosition(1);
                    sleep(1000);
                    tempo.reset();
                    while (tempo.seconds() < 2){
                        MART.setPower(-0.2);
                    }
                    tempo.reset();
                    while (tempo.seconds() < 2.5){
                        MBD.setPower(0.40);
                    }
                    MBD.setPower(0);
                    sleep(500);
                    drive.followTrajectorySequence(midBD);
                    tempo.reset();
                    while (tempo.seconds() < 1){
                        MART.setPower(0.1);
                    }
                    SGD.setPosition(1);
                    MART.setPower(0);
                }
                //RIGHT

                else{
                    drive.followTrajectorySequence(right);
                    tempo.startTime();
                    tempo.reset();
                    while (tempo.seconds() < 1){
                        MART.setPower(0.1);
                    }
                    MART.setPower(0);
                    sleep(1000);
                    SGE.setPosition(1);
                    tempo.reset();
                    while (tempo.seconds() < 1){
                        MART.setPower(-0.3);
                    }
                    MART.setPower(0);

                }

            requestOpModeStop();

        }
    class Pipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(0.0, 0.0, 255);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline rodando");

            Rect leftRect = new Rect(140, 400, 300, 300);
            Rect midRect = new Rect(700, 400, 300, 300);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, midRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(midCrop, midCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];


            telemetry.addData("Direita", midavgfin);
            telemetry.addData("Meio", leftavgfin);
            telemetry.update();

            if (leftavgfin > (midavgfin + 4) || midavgfin > (leftavgfin + 4)){
                if (leftavgfin < midavgfin){
                    ObjectPos = AutonomusTraining.mark.Middle;

                }
                else if (leftavgfin > midavgfin) {

                    ObjectPos = AutonomusTraining.mark.Right;
                }}
            else{
                ObjectPos = AutonomusTraining.mark.Left;
            }

            return (output);
        }
    }
}
                /*braco.reset();
                while (braco.seconds() < 2){
                    MBD.setPower(0.45);
                }
                MBD.setPower(0);
                sleep(1000);
                braco.reset();
                while (braco.seconds() < 2){
                    MART.setPower(0.1);
                }
                MART.setPower(0);
                sleep(1000);
                braco.reset();
                while (braco.seconds() < 1){
                    SGE.setPosition(0.5);
                }
            */