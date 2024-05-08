package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraOpmodes.PropAzul;
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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    boolean meio, direita, esquerda;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

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
                    .lineToLinearHeading(new Pose2d(-20, 2, Math.toRadians(0)))
                    .strafeTo(new Vector2d(-20, -4))
                    .turn(Math.toRadians(180))
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

            TrajectorySequence RightP = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-3, 2, Math.toRadians(0)))
                    .build();

            ElapsedTime tempo = new ElapsedTime();
            SGE.setPosition(0);
            SGD.setPosition(0);
        initOpenCV();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        waitForStart();

        if (!isStopRequested())
                //RIGHT
                if (cX > 570 && cY > 600){
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
                    direita = true;
                }

                //MIDDLE
                //127, 590
                else if (cX < 500 && cX > 100){
                    tempo.reset();
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
                    meio = true;
                }

                //LEFT
                else{
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
                    esquerda = true;
                }

                if (direita){
                    tempo.reset();
                    while (tempo.seconds()<2){
                        MBD.setPower(0.4);
                    }
                    MBD.setPower(0);
                    drive.followTrajectorySequence(RightP);
                    tempo.reset();
                    while (tempo.seconds() <2){
                        MART.setPower(0.2);
                    }
                    SGD.setPosition(1);
                    MART.setPower(0);
                }

            requestOpModeStop();

        }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(new AutonomusTraining.ConeAzul());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class ConeAzul extends OpenCvPipeline {
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

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat YCbCR = new Mat();
            Imgproc.cvtColor(frame, YCbCR, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(63, 70, 56); // the lower hsv threshold for Blue
            Scalar upperYellow = new Scalar(232, 255, 255); // the upper hsv threshold for Blue


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