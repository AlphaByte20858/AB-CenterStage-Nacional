package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "Motores", group = "OpMode")
public class Motores extends OpMode {
    DcMotorEx MEF, MDF, MET, MDT;
    public void init() {
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
    }
    public void loop() {
        if (gamepad1.a){MDT.setPower(1);}
        if (gamepad1.b){MDF.setPower(1);}
        if (gamepad1.x){MET.setPower(1);}
        if (gamepad1.y){MEF.setPower(1);}
        else {MEF.setPower(0);MET.setPower(0);MDF.setPower(0);MDT.setPower(0);}

        telemetry.addLine("MDT == A");
        telemetry.addLine("MDF == B");
        telemetry.addLine("MET == X");
        telemetry.addLine("MEF == Y");
        telemetry.update();}}