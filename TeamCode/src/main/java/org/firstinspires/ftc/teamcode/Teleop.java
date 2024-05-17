package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOp", group = "OpMode")
public class Teleop extends OpMode {
    AnalogInput BE;
    DcMotorEx MEF, MDF, MET, MDT, MBD, MART;
    Servo SGD, SGE, SD;
    IMU imu;
    ElapsedTime GE = new ElapsedTime();
    /* public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 0; // in
    public static double GEAR_RATIO = 0; // output (wheel) speed / input (encoder) speed
    public static double LATERAL_DISTANCE = 0; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel */

    @Override
    public void init() {
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MART = hardwareMap.get(DcMotorEx.class, "Articular");
        MBD = hardwareMap.get(DcMotorEx.class, "MBD");
        SGD = hardwareMap.get(Servo.class,"SGD" );
        SGE = hardwareMap.get(Servo.class,"SGE" );
        SD = hardwareMap.get(Servo.class,"SD" );
        BE = hardwareMap.get(AnalogInput.class, "BE");

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


        MEF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MDF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MET.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MDT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MBD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MART.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu  = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        SGD.setPosition(0);
        SGE.setPosition(0);
        SD.setPosition(0);


    }

    @Override
    public void loop() {
        movi();
        claw();
        arm();
        drone();
        art();
    }
    public void movi() {
        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x * 0.9               ;
        double yaw     =  gamepad1.right_stick_x * 0.6;

        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        YawPitchRollAngles XYZangles = imu.getRobotYawPitchRollAngles();
        AngularVelocity XYZvelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        if(gamepad1.right_bumper){
            MotorsPower(motorEsquerdoFf, motorDireitoFf , motorEsquerdoTf, motorDireitoTf);
        }
        else {
            MotorsPower(motorEsquerdoFf * 0.85, motorDireitoFf * 0.85, motorEsquerdoTf * 0.85, motorDireitoTf * 0.85);
        }
        telemetry.addData("MDF", MDF.getCurrentPosition());
        telemetry.addData("MEF", MEF.getCurrentPosition());
        telemetry.addData("MET", MET.getCurrentPosition());
        telemetry.addData("MDT", MDT.getCurrentPosition());
    }
    public void MotorsPower(double p1, double p2, double p3,double p4){
        MEF.setPower(p1);
        MDF.setPower(p2);
        MET.setPower(p3);
        MDT.setPower(p4);
    }

    //programação da garra
    public void claw() {
        double GA = 0.5;
        double GF = 0;
        double Position = SGE.getPosition();
        GE.startTime();
        if (gamepad2.x && GE.seconds() >= 0.5) {
            if (Position == 0) {
                SGE.setPosition(GA);
                SGD.setPosition(GA);
            }
            else if (Position == GA) {
                SGE.setPosition(GF);
                SGD.setPosition(GF);
            }
            GE.reset();
        }
        if (gamepad2.dpad_right){
            SGD.setPosition(0.5);
        }
        if (gamepad2.dpad_left){
            SGE.setPosition(0.5);
        }
    }
    int Powerteste = 0;
    int lastposArm = 0;
    //Motores do braçoe
    ElapsedTime tempoArm = new ElapsedTime();
    public void arm(){
        MBD.setPower(gamepad2.right_trigger - gamepad2.left_trigger);telemetry.addData("ValorBraço", MBD.getPower());} //1.4

    //Avião
    public void drone(){
        if (gamepad2.a){
            SD.setPosition(0.6);
        }
    }
    int lastPosArt = 0;
    //Articulaçao da garra
    public void art() {
        if (gamepad2.right_bumper) {
            MART.setPower(0.2);
        }
        else if (gamepad2.left_bumper){
            MART.setPower(-0.2);
        }
        else {
            MART.setPower(0);
        }
        telemetry.addData("BE",BE.getVoltage());
        telemetry.addData("art", MART.getCurrentPosition());
    }
}