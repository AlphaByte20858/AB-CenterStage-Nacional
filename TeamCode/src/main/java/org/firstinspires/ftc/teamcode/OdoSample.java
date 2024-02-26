package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "OdoSample", group = "LinearOpMode")
public class OdoSample extends LinearOpMode {
    Servo SGE, SGD;
    DcMotorEx MDF, MDT, MEF, MET;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        SGE = hardwareMap.get(Servo.class, "SGE");
        SGD = hardwareMap.get(Servo.class, "SGD");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");

        MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MET.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        SGE.setPosition(0.0);
        SGD.setPosition(0.0);
        waitForStart();
        while(opModeIsActive()){

        //esquerda vermelho esquerda perto
        if (gamepad1.dpad_left) {
            TrajectorySequence EP = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                    .back(24)

                    .build();
            drive.followTrajectorySequence(EP);

    }
        /*
        NOTAS DE FUTURAS ATUALIZAÇÕES:
         1. Para implementar uso de servos e etc, usar .UNSTABLE_addTemporalMarkerOffset(tempo, () -> servo.setPosition(0))
         2. Define a direção para girar com .setTangent(double)
         3. setReversed(boolean) OBS:auto explicativo
         4. setConstraints(TrajectoryVelocityConstraint, TrajectoryAccelerationConstraint) // E TAMBÉM // resetConstraints()
         5.                         .UNSTABLE_addTemporalMarkerOffset(100, () -> SGE.setPosition(0))

         */
}}}
