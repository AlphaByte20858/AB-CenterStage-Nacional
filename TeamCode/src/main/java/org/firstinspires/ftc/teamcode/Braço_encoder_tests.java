package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Braço_encoder_tests extends OpMode {
    int curPos;
    DcMotorEx MART, MBD;
    double integral = 0;
    double error = 0;
    double TargetPos = 0;
    double derivate = 0;
    double deltaError = 0;
    private double lastError = 0;
    ElapsedTime tempo = new ElapsedTime();
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    public void init(){
        MART = hardwareMap.get(DcMotorEx.class, "MART");
        MBD = hardwareMap.get(DcMotorEx.class, "MBD");

        MBD.setDirection(DcMotorSimple.Direction.REVERSE);

        MART.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MART.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MART.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    double a = 0;
    double b = 0;
    public void loop(){
        if (gamepad2.a){
            MBD.setPower(getPidArm(a + 1000));
        }
        if (gamepad2.b){
            MART.setPower(getPidArt(a + 50));
        }
        if (gamepad2.x){
            MART.setPower(0);
            MBD.setPower(0);
        }
    }

    public void setArmPower(){
        ArmPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }

    public void ArmPower(double power){
        MART.setPower(power);
        MBD.setPower(power);
    }

    public double getPidArm(double pos) {

        // calculo do pid

        TargetPos = MBD.getTargetPosition();

        error = pos - TargetPos;

        integral += error * tempo.seconds();

        deltaError = (error - lastError);

        derivate = deltaError / tempo.seconds();

        lastError = error;

        error = error * 0.01;
        integral = integral * 0.01;
        derivate = derivate * 0.01;

        pidGains.p = error * pidCoeffs.p;
        pidGains.i = integral * pidCoeffs.i;
        pidGains.d = derivate * pidCoeffs.d;

        tempo.reset();

        double output = (pidGains.p + pidGains.i + pidGains.d );
        return output;
    }

    public double getPidArt(double pos) {

        // calculo do pid

        TargetPos = MBD.getTargetPosition();

        error = pos - TargetPos;

        integral += error * tempo.seconds();

        deltaError = (error - lastError);

        derivate = deltaError / tempo.seconds();

        lastError = error;

        error = error * 0.01;
        integral = integral * 0.01;
        derivate = derivate * 0.01;

        pidGains.p = error * pidCoeffs.p;
        pidGains.i = integral * pidCoeffs.i;
        pidGains.d = derivate * pidCoeffs.d;

        tempo.reset();

        double output = (pidGains.p + pidGains.i + pidGains.d );
        return output;
    }
}