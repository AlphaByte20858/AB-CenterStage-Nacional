package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoSample extends OpMode {
    Servo SGD, SGE;
    public void init(){
        SGD = hardwareMap.get(Servo.class,"SGD" );
        SGE = hardwareMap.get(Servo.class,"SGE" );
        SGD.setDirection(Servo.Direction.REVERSE);
        SGE.setDirection(Servo.Direction.FORWARD);
        SGD.setPosition(0.5);
        SGE.setPosition(0.5);
    }
    public void loop(){
        if(gamepad2.x) {
            //SGD.setPosition(0.7);
            SGE.setPosition(0.7);
        }
        if (gamepad2.y) {
            //SGD.setPosition(0.2);
            SGE.setPosition(0.2);
        }
    }
}
