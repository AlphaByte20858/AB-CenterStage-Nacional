package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class Autonomus extends LinearOpMode {

    DcMotorEx MDF, MEF, MDT, MET = null;
    IMU imu;

    @Override
    public void runOpMode() {

        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");

        MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MET.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MET.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotorEx.Direction.FORWARD);

        imu  = hardwareMap.get(IMU.class, "IMU");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motorsEncoderTelemetry();
        motorsPowerTelemetry();
        Movi();

        AutonomousMoveWithEncoder(1, 100, 100, 100, 100);
        sleep(500);
        AutonomousMoveWithEncoder(-1, 100, 100, 100, 100);

        while (opModeIsActive()) {

        }
    }
    public void MotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
    public void Movi() {
        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

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
            MotorsPower(motorEsquerdoFf * 0.7, motorDireitoFf * 0.7, motorEsquerdoTf * 0.7, motorDireitoTf * 0.7);
        }
        telemetry.addData("MDF", MDF.getCurrentPosition());
        telemetry.addData("MEF", MEF.getCurrentPosition());
        telemetry.addData("MET", MET.getCurrentPosition());
        telemetry.addData("MDT", MDT.getCurrentPosition());
    }

    public void AutonomousMoveWithEncoder(double power, int posMEF, int posMDF, int posMET, int posMDT){

        MEF.setTargetPosition(posMEF);
        MDF.setTargetPosition(posMDF);
        MET.setTargetPosition(posMET);
        MDT.setTargetPosition(posMDT);

        MEF.setPower(power);
        MDF.setPower(power);
        MET.setPower(power);
        MDT.setPower(power);

        MEF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MDF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MET.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MDT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
}
