
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class vasilea5v2 extends LinearOpMode {
    //de fapt AutonomousA2
    private OpenCvWebcam webcam;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private double width, height;
    public double lastTime;
    private DcMotorEx motorFR, motorFL, motorBR, motorBL;
    private DcMotorEx ecstensor;
    private DcMotorEx alecsticulator;
    private Servo heater;
    private Servo supramax;
    private Servo claw;

    String varrez = "Dreapta";
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    //merge si pentru F2 acest autonom
    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    @Override
    public void runOpMode() {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Front-Right
        ecstensor      = hardwareMap.get(DcMotorEx.class, "extinsator");
        alecsticulator = hardwareMap.get(DcMotorEx.class,"articulator");
        heater      = hardwareMap.servo.get("hitter");
        supramax = hardwareMap.servo.get("articulatie");

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        alecsticulator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ecstensor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        alecsticulator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ecstensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        alecsticulator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ecstensor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetime(-200,1,ecstensor,2000);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        while (!isStarted() && !isStopRequested()) {
            try {
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
            }
        }
        if(!isStopRequested()) {
            Autonom.start();
        }
        while(!isStopRequested()){

        }
    }
    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            Translatare(-130,0,0.3);
            kdf(200);
            Translatare(0,-70,0.3);
            kdf(300);
            heater.setPosition(0.95);
            kdf(300);
            Rotire(-70,0.3);
            kdf(200);
            target(-1130,1,ecstensor);
            kdf(500);
            target(-380,1,alecsticulator);
            kdf(250);
            Translatare(0,-50,0.3);
            kdf(200);
            heater.setPosition(0.8);
            kdf(800);
            supramax.setPosition(1);
            kdf(1500);
            Rotire(70,0.3);
            kdf(500);
            target(-260,1,alecsticulator);
            kdf(1500);
            target(-380,1,alecsticulator);
            if(varrez=="Dreapta"&&!isStopRequested()) {
                Translatare(280,0,0.3);
            }
            if(varrez == "Mijloc"&&!isStopRequested()) {
                Translatare(140,0,0.3);
            }
        }
    });
    public void testing(ContourPiepline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 235);
        crThreshHigh = inValues(crThreshHigh, 0, 235);
        cbThreshLow = inValues(cbThreshLow, 0, 235);
        cbThreshHigh = inValues(cbThreshHigh, 0, 235);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void RotireKindaSmooth(int poz, double power, int choice){
        if(choice%4==0) {
            motorFR.setTargetPosition(poz);

            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFR.setPower(power);
        }
        else if(choice%4==1) {
            motorFL.setTargetPosition(poz);

            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFL.setPower(power);
        }
        else if(choice%4==2) {
            motorBR.setTargetPosition(poz);

            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBR.setPower(power);
        }
        else if(choice%4==3) {
            motorBL.setTargetPosition(poz);

            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBL.setPower(power);
        }
    }
    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBL = currentmotorBL + (int) (deltaA * cpdeg);
        targetBR = currentmotorBR + (int) (deltaA * cpdeg);
        targetFL = currentmotorFL + (int) (deltaA * cpdeg);
        targetFR = currentmotorFR + (int) (deltaA * cpdeg);

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }
    }
    public void target(int poz, double pow, DcMotorEx motor){
        motor.setTargetPosition(poz);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        while (motor.isBusy());
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void targetime(int poz, double pow, DcMotorEx motor,int t){
        double lastTime2;
        lastTime2 = System.currentTimeMillis();
        while(lastTime2 + t > System.currentTimeMillis()) {
            motor.setTargetPosition(poz);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
            while (motor.isBusy()) ;
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void kdf(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime + t < System.currentTimeMillis()){

        }
    }
}
/*              |
                |
                |
                |
                |
________________|________________
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |

 */

    /*
    H/W(big=1):1.588
    H/W(small=3):0.23
    H/W(medium=2):4.23
     */