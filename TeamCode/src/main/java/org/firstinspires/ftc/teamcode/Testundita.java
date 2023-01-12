package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import static org.firstinspires.ftc.teamcode.pid.kp;
import static org.firstinspires.ftc.teamcode.pid.ki;
import static org.firstinspires.ftc.teamcode.pid.kd;
import static org.firstinspires.ftc.teamcode.pid.iarm;
import static org.firstinspires.ftc.teamcode.pid.darm;
import static org.firstinspires.ftc.teamcode.pid.parm;
import static org.firstinspires.ftc.teamcode.pid.p;
import static org.firstinspires.ftc.teamcode.pid.i;
import static org.firstinspires.ftc.teamcode.pid.d;
import static org.firstinspires.ftc.teamcode.pid.pslider;
import static org.firstinspires.ftc.teamcode.pid.islider;
import static org.firstinspires.ftc.teamcode.pid.dslider;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class Testundita extends OpMode {

    private DcMotorEx undita;
    private DcMotorEx articulatie_undita;
    private boolean stop=false;

    public void init()
    {
        undita=hardwareMap.get(DcMotorEx.class, "undita");
        articulatie_undita=hardwareMap.get(DcMotorEx.class, "articulatie_undita");

        undita.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        articulatie_undita.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        articulatie_undita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        undita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        undita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        articulatie_undita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
    @Override
    public void start(){
        Systems.start();

    }
    private final Thread Systems = new Thread(new Runnable()
    {
        @Override
        public void run(){
            while(!stop){
                undita.setPower(gamepad1.right_stick_y);
                articulatie_undita.setPower(gamepad1.right_stick_x);
            }
        }
    });
    public void stop(){stop = true;}
    public void loop(){}
}