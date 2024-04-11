package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous (name = "Autoninho", group = "LinearOpMode")
public class AutonomusTraining extends LinearOpMode {
        DcMotorEx MET, MEF, MDT, MDF;
        ElapsedTime eita;

        public void runOpMode() {

            MET = hardwareMap.get(DcMotorEx.class, "MET");
            MDT = hardwareMap.get(DcMotorEx.class, "MDT");
            MDF = hardwareMap.get(DcMotorEx.class, "MDF");
            MEF = hardwareMap.get(DcMotorEx.class, "MEF");

            MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            MEF.setDirection(DcMotorEx.Direction.REVERSE);
            MDF.setDirection(DcMotorEx.Direction.FORWARD);
            MET.setDirection(DcMotorEx.Direction.REVERSE);
            MDT.setDirection(DcMotor.Direction.FORWARD);


            MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            TrajectorySequence sla = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .forward(22)
                    .build();

            waitForStart();

            eita.startTime();
            if (gamepad1.a && eita.seconds() > 3){
                drive.followTrajectorySequence(sla);
            }
        }
}