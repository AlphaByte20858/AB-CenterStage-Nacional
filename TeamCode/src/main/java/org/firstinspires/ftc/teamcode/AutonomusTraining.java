package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous (name = "Autoninho", group = "LinearOpMode")
public class AutonomusTraining extends LinearOpMode {
        DcMotorEx MET, MEF, MDT, MDF, MBD, MART;
        Servo SGE, SGD;

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

            MDF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MDT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MEF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MET.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MBD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            MART.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            MBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            TrajectorySequence mid = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-24, -4, Math.toRadians(0)))
                    .turn(Math.toRadians(180))
                    .build();

            TrajectorySequence midBD = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    //.turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-24, 36, Math.toRadians(-90)))
                    .build();

            TrajectorySequence ED = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .strafeRight(25)
                    .build();


            ElapsedTime tempo = new ElapsedTime();
            SGE.setPosition(0.5);
            SGD.setPosition(0.5);
            waitForStart();


            if (!isStopRequested())
                //drive.followTrajectorySequence(Stacionar);

                //SGE.setPosition(0);
                drive.followTrajectorySequence(mid);
            sleep(2000);
            tempo.reset();
            while (tempo.seconds() < 3){
                MART.setPower(0.2);
            }
            MART.setPower(0);
            sleep(500);
            tempo.reset();
            while (tempo.seconds() < 1){
                SGD.setPosition(0);
            }
            sleep(500);
            tempo.reset();
            while (tempo.seconds() < 2){
                MART.setPower(-0.2);
            }
            tempo.reset();
            while (tempo.seconds() < 2){
                MBD.setPower(0.40);
            }
            MBD.setPower(0);
            sleep(500);
            drive.followTrajectorySequence(midBD);
            tempo.reset();
            while (tempo.seconds() < 2){
                MART.setPower(0.2);
            }
            while (3 > tempo.seconds() && tempo.seconds() > 1.8){
                SGE.setPosition(0);
            }
            MART.setPower(0);

            requestOpModeStop();
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