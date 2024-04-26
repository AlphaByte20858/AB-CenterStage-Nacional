package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TestesOP", group = "OpMode")
public class TestesFast extends OpMode {
    Servo SGE, SGD;
    public void init() {
        SGE = hardwareMap.get(Servo.class, "SGE");
        SGD = hardwareMap.get(Servo.class, "SGD");
        SGE.setPosition(0);
        SGD.setPosition(0);
    }

    public void loop() {

        if (gamepad1.a){
            SGE.setPosition(0.5);
        }
        else if (gamepad1.b){
            SGD.setPosition(0.5);
        }
    }
}
