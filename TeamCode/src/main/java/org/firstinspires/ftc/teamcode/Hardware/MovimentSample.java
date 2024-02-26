package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Movimento Teste", group = "OpMode")

public class MovimentSample extends OpMode {

    // criação dos mostores
    DcMotorEx MDF, MEF, MDT, MET = null;

    @Override
    public void init() {
        MEF = hardwareMap.get(DcMotorEx.class, "LeftDriveUp");
        MDF = hardwareMap.get(DcMotorEx.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotorEx.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotorEx.class, "RightDriveDown");

        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop(){
        TeleOpMovi();
    }

    public void MotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }

    public void MotorsPowerall(double speed){
        MEF.setPower(speed);
        MDF.setPower(speed);
        MET.setPower(speed);
        MDT.setPower(speed);
    }

    public void ModeMotor(DcMotor.RunMode mode){
        MDF.setMode(mode);
        MDT.setMode(mode);
        MEF.setMode(mode);
        MET.setMode(mode);
    }

    public void AutonomousMoveWithEncoder(double power, int posMEF, int posMDF, int posMET, int posMDT){
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MEF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MEF.setTargetPosition(posMEF);
        MDF.setTargetPosition(posMDF);
        MET.setTargetPosition(posMET);
        MDT.setTargetPosition(posMDT);

        MEF.setPower(power);
        MDF.setPower(power);
        MET.setPower(power);
        MDT.setPower(power);

        MEF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MET.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void motorsEncoderTelemetry(){
        telemetry.addData("A potencia do motorEsquerdoF é de:", MEF.getCurrentPosition());
        telemetry.addData("A potencia do motorDireitoF é de:", MDF.getCurrentPosition());
        telemetry.addData("A potencia do motorEsquerdoT é de:", MET.getCurrentPosition());
        telemetry.addData("A potencia do motorDireitoT é de:", MDT.getCurrentPosition());
        telemetry.update();
    }

    public void motorsPowerTelemetry(){
        telemetry.addData("A potencia do motorEsquerdoF é de:", MEF.getPower());
        telemetry.addData("A potencia do motorDireitoF é de:", MDF.getPower());
        telemetry.addData("A potencia do motorEsquerdoT é de:", MET.getPower());
        telemetry.addData("A potencia do motorDireitoT é de:", MDT.getPower());
        telemetry.update();
    }

    public void TeleOpMovi() {
        double axial = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw = Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);

        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);
        MotorsPower(motorEsquerdoFf, motorDireitoFf, -motorEsquerdoTf , motorDireitoTf);
    }

    public void teleopNew(){
        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);

        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);
        if(gamepad1.right_bumper){
            MotorsPower(motorEsquerdoFf, motorDireitoFf , motorEsquerdoTf, motorDireitoTf);
        }
        else {
            MotorsPower(motorEsquerdoFf * 0.7, motorDireitoFf * 0.7, motorEsquerdoTf * 0.7, motorDireitoTf * 0.7);
        }
    }
    public void TeleOpSlowMove(){
        if(gamepad1.dpad_down){
            MotorsPower(-0.5,-0.5,-0.5,-0.5);
        }

        // Seta de cima
        else if(gamepad1.dpad_up){
            MotorsPower(0.5,0.5,0.5,0.5);
        }

        // Seta da direita
        else if(gamepad1.dpad_right){
            MotorsPower(0.5,-0.5,-0.5,0.5);
        }

        // Seta da esquerda
        else if(gamepad1.dpad_left){
            MotorsPower(-0.5,0.5,0.5,-0.5);
        }
    }
}