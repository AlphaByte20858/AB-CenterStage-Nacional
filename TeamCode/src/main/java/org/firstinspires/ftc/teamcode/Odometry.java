package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp (name = "Odonometria", group = "OpMode")
public class Odometry extends OpMode {
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public void init() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(11.52, -61.49, Math.toRadians(90)))
                .splineTo(new Vector2d(12.08, -36.97), Math.toRadians(88.71))
                .build();
    }

    public void loop(){
        Valores();
    }
    public enum Prop{
        Esquerda,
        Direita,
        Frente,
        NoFound
    }
    public void Valores() {
        telemetry.addData("Valor da roda esquerda:", leftEncoder.getCurrentPosition());
        telemetry.addData("Valor da roda direita:", rightEncoder.getCurrentPosition());
        telemetry.addData("Valor da roda Traseira", frontEncoder.getCurrentPosition());
        telemetry.update();
    }

}
/*calculo para definir posição do robô
        TrajectorySequence j2 = drive.trajectorySequenceBuilder(j1.end())


                .strafeLeft(105)
                .strafeRight(18)
                .turn(Math.toRadians(59))
                .addDisplacementMarker(3, () -> {
                    curArm = 3100;
                    Arm.setTargetPosition(curArm);
                    Arm.setPower(pidLinear(0.9));
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .forward(14)
                .waitSeconds(0.1)
                .addTemporalMarker(25, () -> {
                    servoMotor.setPosition(0.70);
                })
                .build();


 */