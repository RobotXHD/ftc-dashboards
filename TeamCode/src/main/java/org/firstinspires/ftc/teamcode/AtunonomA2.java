package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AtunonomA2 extends LinearOpMode{
    private OpenCvWebcam webcam;
    private double width, height;
    public double lastTime;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private DcMotorEx motorFR, motorFL, motorBR, motorBL;
    private DcMotorEx ecstensor;
    private DcMotorEx alecsticulator;
    private Servo heater;
    private Servo supramax;
    String varrez = "Dreapta";
    FunctiiDeMiscare a = new FunctiiDeMiscare(true);
    @Override
    public void runOpMode() throws InterruptedException {
        a.Init(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        */
        while (!isStarted() && !isStopRequested()) {
            /*try {
                width = pipeline.getRect().width;
                height = pipeline.getRect().height;
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.addData("Rectangle Width:", width);
                telemetry.addData("Rectangle Height:", height);
                telemetry.addData("Rectangle H/W:", height / width);
                telemetry.addData("heater:", heater.getPosition());
                if (height / width > 1.2 && height / width < 2.4) {
                    telemetry.addData("Rect", "1");
                    varrez = "Stanga";
                } else if (height / width > 2.4) {
                    telemetry.addData("Rect", "2");
                    varrez = "Mijloc";
                } else {
                    telemetry.addData("Rect", "3");
                    varrez = "Dreapta";
                }
                telemetry.addData("caz", varrez);
                telemetry.update();
            }
            catch (Exception E) {
                telemetry.addData("Webcam error", "Please restart");
            }*/
        }
        if(!isStopRequested()) {
            a.Translatare(140,0,0.5);
            Autonom.start();
        }
        while(!isStopRequested()){

        }
    }
    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            if(varrez=="Stanga"&&!isStopRequested()) {
                a.Translatare(140,0,0.5);
                a.kdf(200);
                a.Translatare(0,-245,0.5);
            }
            if(varrez == "Mijloc"&&!isStopRequested()) {
                a.Translatare(140,0,0.5);
                a.kdf(200);
                a.Translatare(0,-245,0.5);
                a.kdf(600);
                a.Translatare(-140,0,0.5);
            }

            if(varrez == "Dreapta"&&!isStopRequested()) {
                a.Translatare(-140,0,0.5);
                a.kdf(200);
                a.Translatare(0,-245,0.5);
            }
        }
    });
}
