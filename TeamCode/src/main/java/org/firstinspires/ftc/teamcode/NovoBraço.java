package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "A", group = "OpMode")
public class NovoBraço extends OpMode {

    DcMotorEx MRL;
    ServoImplEx gaby;

    public void init() {MRL = hardwareMap.get(DcMotorEx.class, "MRL");gaby = hardwareMap.get(ServoImplEx.class, "gaby");}
    public void loop() {movi();servo();Encoder();Valores();}
    public void movi() {if (gamepad1.y){MRL.setPower(1);}else{MRL.setPower(0);}}
    public void servo() {if (gamepad2.x){gaby.setPosition(0.618);}else{gaby.setPosition(0.314);}}
    public void Encoder() {}
    public void Valores() {}

}