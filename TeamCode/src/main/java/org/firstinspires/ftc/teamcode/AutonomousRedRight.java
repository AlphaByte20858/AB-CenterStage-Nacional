package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "RedProp", group = "LinearOpMode")
public class AutonomousRedRight extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    DcMotorEx MEF, MDF, MET, MDT, MBD, MART,BE;
    Servo SGD, SGE, SD;

    public void runOpMode() {
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        BE = hardwareMap.get(DcMotorEx.class, "BE");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MART = hardwareMap.get(DcMotorEx.class, "Articular");
        MBD = hardwareMap.get(DcMotorEx.class, "MBD");
        SGD = hardwareMap.get(Servo.class,"SGD" );
        SGE = hardwareMap.get(Servo.class,"SGE" );
        SD = hardwareMap.get(Servo.class,"SD" );

        MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotorEx.RunMode   .STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MART.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MBD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BE.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MART.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MDF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MBD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BE.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MET.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);


        MART.setDirection(DcMotorEx.Direction.REVERSE);
        MBD.setDirection(DcMotorEx.Direction.FORWARD);
        SGD.setDirection(Servo.Direction.REVERSE);
        SD.setDirection(Servo.Direction.REVERSE);
        BE.setDirection(DcMotorSimple.Direction.REVERSE);

        MBD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MART.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SGD.setPosition(0);
        SGE.setPosition(0);
        SD.setPosition(0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        boolean min = cX > 265 && cY > 585;
        boolean max = cX < 275 && cY < 595;
        ElapsedTime tempo = new ElapsedTime();
        waitForStart();

        controlHubCam.stopStreaming();

        TrajectorySequence MpontuarMark = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(-40,40, Math.toRadians(-95)))
                .lineToLinearHeading(new Pose2d(-22,0,Math.toRadians(0)))
                .turn(Math.toRadians(-100))
                .build();


        if (!isStopRequested())
            drive.followTrajectorySequence(MpontuarMark);
            tempo.reset();
            while(tempo.seconds() < 0.5){
                MART.setTargetPosition(-100);
                MART.setPower(0.4);
                MART.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            MART.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(tempo.seconds() < 0.6){
            }
            SGE.setPosition(0.5);
            tempo.reset();
            while(tempo.seconds() < 5){

            }

            TrajectorySequence Estacionar = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .turn(Math.toRadians(180))
                    .forward(28)
                    .turn(90)
                    .forward(20)
                    .build();
        }

        // Release resource

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(new AutonomousRedRight.ConeVermelho());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class ConeVermelho extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

                telemetry.addData("X", cX);
                telemetry.addData("Y", cY);
                telemetry.update();
            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat YCbCR = new Mat();
            Imgproc.cvtColor(frame, YCbCR, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100); // the lower hsv threshold for red
            Scalar upperYellow = new Scalar(255, 255, 180); // the upper hsv threshold for red


            Mat yellowMask = new Mat();
            Core.inRange(YCbCR, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }

    private static double getDistance(double width) {
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

}