/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="teste", group="Linear OpMode")
//@Disabled
public class OpenCv extends LinearOpMode {

    OpenCvWebcam webcam; //declarar a webcam
    String linha;

    @Override
    public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //mapeamento da webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); //declaração do id da webcam

        webcam.setPipeline(new Pipeline()); //definindo o pipeline

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(720, 1280, OpenCvCameraRotation.UPRIGHT); //definindo o tamanho de tela da sua câmera
            } //logitech c920 fullHD

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
    }

    class Pipeline extends OpenCvPipeline{  //contrução do piperline
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);
        //função proscess frame, irá detectar a localização geral de um objeto
        public Mat processFrame(Mat input){
            // e dividir o nosso espaço de vizualização em 3 quadrantes
            //parametros de altura e largura de cada quadrante
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb); //converter o feedback da camera para YCbCr, deixa o desempenho melhor por otimiza-lo
            telemetry.addLine("Pipeline rodando");

            Rect leftRect = new Rect(1, 1, 639, 1079);
            Rect rightRect = new Rect(1280, 1, 639, 1079);
            Rect midRect = new Rect(640, 1, 639, 1079);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2); //esse código permite voce vizualizar os
            //quadrantes na sua driver hub, muito util para testes e verificação
            Imgproc.rectangle(output, midRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);
            Core.extractChannel(midCrop, midCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];  //linhas de escaneamento
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            telemetry.addData("Esquerda", leftavgfin);
            telemetry.addData("Meio", midavgfin);
            telemetry.addData("Direita", rightavgfin);
            telemetry.update();

            if (leftavgfin > rightavgfin && leftavgfin > midavgfin){ // linhas para printar em qual quadrante o
                //objeto está sendo identificado
                telemetry.addLine("Esquerda");
                linha = "Esquerda";
            } else if (rightavgfin > midavgfin){
                telemetry.addLine("Direita");
                linha = "Direita";
            } else{
                telemetry.addLine("Meio");
                linha = "Meio";
            }

            return (output);
        }
    }
}