package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous (name = "Twopixxellong", group = "LinearOpMode")
public class TWOPIXELSBLonge extends LinearOpMode {

    DcMotorEx MEF, MDF, MET, MDT, MBD, MART;
    Servo SGD, SGE, SD;

    public void runOpMode() {

        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MART = hardwareMap.get(DcMotorEx.class, "Articular");
        MBD = hardwareMap.get(DcMotorEx.class, "MBD");
        SGD = hardwareMap.get(Servo.class,"SGD" );
        SGE = hardwareMap.get(Servo.class,"SGE" );
        SD = hardwareMap.get(Servo.class,"SD" );

        MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MART.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MBD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MART.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MDF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MBD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        ElapsedTime tempo = new ElapsedTime();
        waitForStart();


        TrajectorySequence MpontuarMark = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(-40,40, Math.toRadians(-95)))
                .lineToLinearHeading(new Pose2d(-24,0,Math.toRadians(0)))
                .turn(Math.toRadians(90))
                .build();

        Trajectory pontuar= drive.trajectoryBuilder(MpontuarMark.end())
                .lineToLinearHeading(new Pose2d(-24,50,Math.toRadians(90)))
                .build();

        if (!isStopRequested())
            drive.followTrajectorySequence(MpontuarMark);
        sleep(22000);
        drive.followTrajectory(pontuar);
        requestOpModeStop();    }


}