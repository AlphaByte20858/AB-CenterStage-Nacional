/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.CameraOpmodes;

import android.annotation.SuppressLint;
import android.provider.ContactsContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autoRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RedAuto", group="Linear OpMode")
//@Disabled
public class WebCam extends LinearOpMode {

    OpenCvWebcam webcam = null;
    String pos;
        DcMotorEx MEF, MDF, MET, MDT, MBD, MART, LE, FE;
        Servo SGD, SGE, SD;

    public enum mark{
        Left,
        Right,
        Middle
    }
    mark ObjectPos;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new Pipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MART = hardwareMap.get(DcMotorEx.class, "Articular");
        MBD = hardwareMap.get(DcMotorEx.class, "MBD");
        SGD = hardwareMap.get(Servo.class,"SGD" );
        SGE = hardwareMap.get(Servo.class,"SGE" );
        SD = hardwareMap.get(Servo.class,"SD" );
        LE = hardwareMap.get(DcMotorEx.class, "LE");
        FE = hardwareMap.get(DcMotorEx.class, "FE");

        MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MART.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MBD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LE.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FE.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MART.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MDF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MBD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FE.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LE.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MET.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);

        MART.setDirection(DcMotorEx.Direction.REVERSE);
        MBD.setDirection(DcMotorEx.Direction.FORWARD);
        SGD.setDirection(Servo.Direction.REVERSE);
        SD.setDirection(Servo.Direction.REVERSE);

        MBD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MART.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SGD.setPosition(0);
        SGE.setPosition(0);
        SD.setPosition(0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        waitForStart();
        TrajectorySequence MpontuarMark = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(-40,40, Math.toRadians(-95)))
                .strafeLeft(5)
                .lineToLinearHeading(new Pose2d(-22,5,Math.toRadians(0)))
                .turn(Math.toRadians(-150))
                .back(2)
                .build();
        TrajectorySequence Frente = drive.trajectorySequenceBuilder(MpontuarMark.end())
                //.lineToLinearHeading(new Pose2d(-40,40, Math.toRadians(-95)))
                .forward(10)
                .build();



        TrajectorySequence LpontuarMark = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .strafeLeft(5)
                .lineToLinearHeading(new Pose2d(-24,5,Math.toRadians(0)))
                .turn(Math.toRadians(-95))
                .back(2)
                .build();

        TrajectorySequence DpontuarMark = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .strafeLeft(5)
                .lineToLinearHeading(new Pose2d(-24,5,Math.toRadians(0)))
                .turn(Math.toRadians(95))
                .back(2)
                .build();

        ElapsedTime tempo = new ElapsedTime();
        if (!isStopRequested());
        switch (ObjectPos){
            case Left:
                drive.followTrajectorySequence(LpontuarMark);
                tempo.reset();
                while(tempo.seconds() < 2){
                    MART.setPower(-0.05);
                }
                MART.setPower(0);
                sleep(1000);
                SGE.setPosition(0.5);
                tempo.reset();
                while(tempo.seconds() < 1){}
                tempo.reset();
                while(tempo.seconds() < 2){
                    MART.setPower(0.05);
                }
                MART.setPower(0);
                tempo.reset();
                while(tempo.seconds() > 1){}
                requestOpModeStop();

            case Middle:
                drive.followTrajectorySequence(MpontuarMark);
                tempo.reset();
                while(tempo.seconds() < 2){
                    MART.setPower(-0.05);
                }
                MART.setPower(0);
                sleep(1000);
                SGE.setPosition(0.5);
                tempo.reset();
                while(tempo.seconds() < 1){}
                tempo.reset();
                while(tempo.seconds() < 2){
                    MART.setPower(0.05);
                }
                MART.setPower(0);
                tempo.reset();
                while(tempo.seconds() > 1){}
                requestOpModeStop();
            case Right:
                drive.followTrajectorySequence(DpontuarMark);
                tempo.reset();
                while(tempo.seconds() < 2){
                    MART.setPower(-0.05);
                }
                MART.setPower(0);
                sleep(1000);
                SGE.setPosition(0.5);
                tempo.reset();
                while(tempo.seconds() < 1){}
                tempo.reset();
                while(tempo.seconds() < 2){
                    MART.setPower(0.05);
                }
                MART.setPower(0);
                tempo.reset();
                while(tempo.seconds() > 1){}
                requestOpModeStop();
        }



    }
    public void resetOdo(){
        FE.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LE.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FE.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LE.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        Scalar rectColor = new Scalar(255, 0.0, 0.0);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline rodando");

            Rect leftRect = new Rect(140, 400, 300, 300);
            Rect midRect = new Rect(850, 400, 300, 300);

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

            if (leftavgfin > (midavgfin + 7) || midavgfin > (leftavgfin + 7)){
                if (leftavgfin < midavgfin){
                    ObjectPos = mark.Right;

                }
                else if (leftavgfin > midavgfin) {

                    ObjectPos = mark.Middle;
                }}
            else{
                ObjectPos = mark.Left;
            }

            return (output);
        }
    }
}