package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class alecs extends OpMode {
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx ecstensor;
    private DcMotorEx alecsticulator;
    private Servo heater;
    private Servo supramax;
    double sm = 1, ms = 2;
    private BNO055IMU imu;
    double y, x, rx, rx2;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double colagen=0;
    private boolean alast = false,lblast = false,rblast = false;
    boolean v = true,ok1,ok2=false,ok3,ok4,ok5,ok6,ok7;
    private boolean stop=false,setSetpoint=false,setsetSetpoint=false;
    int okGrip = 1, okClaw = 1;
    public int i;
    public int cn=0;
    private double correction=0;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25, lbcn=0,rbcn=0,acn=0;
    int loaderState = -1;
    private int apoz = 0;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0.0,0.0,0.0);
    Pid_Controller_Adevarat pidslider = new Pid_Controller_Adevarat(0.0,0.0,0.0);
    private long spasmCurrentTime = 0;
    private long pidTime = 0;
    public double difference,medie;
    public double medii[] = new double[10];
    public boolean rotating = false;
    public double realAngle, targetAngle;
    private double forward, right, clockwise;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Front-Right
        ecstensor      = hardwareMap.get(DcMotorEx.class, "extinsator");
        alecsticulator = hardwareMap.get(DcMotorEx.class,"articulator");
        heater      = hardwareMap.servo.get("hitter");
        supramax = hardwareMap.servo.get("articulatie");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

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

        heater.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();
    }
    public void start(){
        Chassis.start();
        Systems.start();
    }
    public void stop(){stop = true;}
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            while(!stop) {
                if(gamepad2.left_bumper) {
                    ok1 = false;
                    ok1 = true;
                }
                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x * 1.5;
                rx = gamepad1.right_stick_x;
                rx2 = gamepad2.right_stick_x/3;
                /*
                pid.setPID(constants.pGyro,constants.iGyro,constants.dGyro);
                if(clockwise != 0.0){
                    correction = 0.0;
                    rotating = true;
                }
                else{
                    if((forward != 0.0 || right != 0.0) && Math.abs(medie) < 0.5) {
                        if (rotating) {
                            targetAngle = realAngle;
                            rotating = false;
                            pid.setSetpoint(targetAngle);
                        }
                        correction = pid.performPID(realAngle);
                    }
                    else{
                        correction = 0.0;
                    }
                }*/
                pmotorFL = y + x - rx - rx2;
                pmotorBL = y - x - rx - rx2;
                pmotorBR = y + x + rx + rx2;
                pmotorFR = y - x + rx + rx2;

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
                if (gamepad1.left_bumper != lblast) {
                    lbcn++;
                }
                if(lbcn>=4){
                    lbcn = 0;
                }
                if(lbcn==2) {
                    sm = 3;
                }
                else if(lbcn==0){
                    sm=1;
                }
                if (gamepad1.right_bumper != rblast) {
                    rbcn++;
                }
                if(rbcn>=4){
                    rbcn = 0;
                }
                if(rbcn==2) {
                    sm = 5;
                }
                else if(rbcn==0){
                    sm=1;
                }
                lblast = gamepad1.left_bumper;
                rblast = gamepad1.right_bumper;
                if(sm==3){
                    POWER(pmotorFR/1.5, pmotorFL/1.5, pmotorBR/1.5, pmotorBL/1.5);
                }
                else if(sm==5){
                    POWER(pmotorFR / 3, pmotorFL / 3, pmotorBR / 3, pmotorBL / 3);
                }
                else{
                    POWER(pmotorFR * 2, pmotorFL * 2, pmotorBR * 2, pmotorBL * 2);
                }
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                //if(ecstensor.getCurrentPosition() < 300) {
                    ecstensor.setPower(gamepad2.left_stick_y);
                /*}
                else{
                    if(gamepad2.left_stick_y > 0) {
                        ecstensor.setPower(0);
                    }
                    else{
                        ecstensor.setPower(gamepad2.left_stick_y / 2);
                    }
                }*/
                if (gamepad2.a != alast) {
                    acn++;
                }
                if(acn>=4){
                    acn = 0;
                }
                if(acn==2) {
                    ms = 2;
                }
                else if(acn==0){
                    ms = 1;
                }
                alast = gamepad2.a;
                alecsticulator.setPower(gamepad2.right_stick_y / 2);
                supramax.setPosition(1-gamepad2.right_trigger);
                if(gamepad2.b){
                    target(300,1,ecstensor);
                }
                if(gamepad2.left_bumper && colagen <= 0.99) {
                    colagen+=0.003;
                }
                if(gamepad2.right_bumper && colagen >= 0.01){
                    colagen-=0.003;
                }
                heater.setPosition(1-colagen);
            }
        }
    });
    @Override
    public void loop() {
        telemetry.addData("motorBL:", motorBL.getCurrentPosition());
        telemetry.addData("motorBR:", motorBR.getCurrentPosition());
        telemetry.addData("motorFL:", motorFL.getCurrentPosition());
        telemetry.addData("motorFR:", motorFR.getCurrentPosition());
        telemetry.addData("ecstensor:", ecstensor.getCurrentPosition());
        telemetry.addData("alecsticulator:", alecsticulator.getCurrentPosition());
        telemetry.addData("supramax:", supramax.getPosition());
        telemetry.addData("heater:", heater.getPosition());
        telemetry.addData("pozitie gheara", gamepad2.right_trigger);
        telemetry.addData("sm:",sm);
        telemetry.addData("ms:",ms);
        telemetry.update();
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
    public void target(int poz, double pow, DcMotorEx motor){
        motor.setTargetPosition(poz);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        while (motor.isBusy());
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
