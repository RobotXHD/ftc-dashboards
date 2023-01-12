
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pid.kp;
import static org.firstinspires.ftc.teamcode.pid.ki;
import static org.firstinspires.ftc.teamcode.pid.kd;
//import static org.firstinspires.ftc.teamcode.pid.kf;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static java.lang.Math.abs;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class PPNotWorking2 extends OpMode{
    //private Gyroscope imu;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor slider;
    private DcMotor slider2;
    private DcMotor arm;
    private DcMotor arm2;
    private Servo claw;
    private Servo sula;
    double sm = 1, ms = 2;
    double poz = 0;
    double gpoz = 0.5;
    double y, x, rx;
    double sliderPower,lastsliderPower,lastGamepadSlider, armPower;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double lastTime;
    float right_stick2;
    float right_stick1;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0.0,0.0,0.0);
    boolean v = true,ok1,ok2,ok3,ok4,ok5,ok6,ok7;
    boolean FirstTime = true;
    boolean inchis = false;
    boolean overpower = true;
    boolean permisie = true;
    boolean stopDJ = false;
    boolean tru=false;
    private boolean stop=false;
    int okGrip = 1;
    private double correction;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25;
    int loaderState = -1;


    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorBL = hardwareMap.get(DcMotor.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "motorFR"); // Motor Front-Right
        slider = hardwareMap.get(DcMotor.class, "slider");
        slider2 = hardwareMap.get(DcMotor.class, "slider2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        claw = hardwareMap.servo.get("claw");
        sula = hardwareMap.servo.get("sula");

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
    }
    @Override
    public void start(){
        Chassis.start();
        Systems.start();
    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            while(!stop) {
                if(gamepad2.left_bumper) {
                    ok1 = false;
                    ok1 = true;
                }
                tru = true;

                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x * 1.5;
                rx = gamepad1.right_stick_x;

                pmotorFL = -y - x - rx;
                pmotorBL = -y + x - rx;
                pmotorBR = -y - x + rx;
                pmotorFR = -y + x + rx;

                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }

                //SLOW-MOTION
                if (gamepad1.left_bumper) {
                    sm = 3;
                }
                else if (gamepad1.right_bumper) {
                    sm = 5;
                }
                else {
                    sm = 0.5;
                }
                if(sm==3){
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                }
                else if(sm==5){
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                }
                else{
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                }
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if(gamepad2.right_bumper)
                    ms = 2;
                else if (gamepad2.left_bumper)
                    ms = 5;
                else ms = 0.5;
                if (gamepad2.a)
                    claw.setPosition(0.2);
                if (gamepad2.y)
                    claw.setPosition(0.4);
                if (gamepad2.b)
                    sula.setPosition(1.0);
                if (gamepad2.x)
                    sula.setPosition(0.0);
                armPower  = gamepad2.left_stick_y * 0.5;
                arm.setPower(armPower / ms);
                arm2.setPower(-armPower / ms );
                sliderPower  = gamepad1.right_stick_y;
                slider.setPower(sliderPower / ms);
                slider2.setPower(sliderPower / ms);
                /*if(gamepad2.b) {
                    slider.setTargetPosition(-1680);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(0.7);
                    while (slider.isBusy() && ok1 == true) ;
                    slider.setPower(0);
                    slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }*/
                /*if(gamepad2.x) {
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(0.3);
                    while (slider.isBusy()&&ok1==true) ;
                    slider.setPower(0);
                    slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }*/
            }
        }
    });
    public void stop(){stop = true;}
    public void loop(){
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Pozitie slider", slider.getCurrentPosition());
        telemetry.addData("Controller Values slider:", gamepad2.right_stick_y);
        telemetry.addData("Controller Values arm:", gamepad2.left_stick_y);
        telemetry.addData("Poz: ", poz);
        telemetry.addData("inchis: ", inchis);
        telemetry.addData("permisie: ", permisie);
        telemetry.addData("asdf: ", gamepad1.right_stick_y);
        telemetry.addData("thread: ", stop);
        telemetry.addData("gheara: ", claw.getPosition());
        telemetry.addData("last slider position:", lastsliderPower);
        telemetry.update();
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}

