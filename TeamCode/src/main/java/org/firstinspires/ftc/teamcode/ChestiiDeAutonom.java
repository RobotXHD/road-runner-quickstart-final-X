package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ChestiiDeAutonom extends LinearOpMode {
    public HardwareMap hwmap = null;
    public OpenCvCamera webcam;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public DcMotorEx sliderR, sliderL, brat;
    public Servo ghearaR, ghearaL, servodubios;
    public Boolean startThread;
    public TouchSensor sliderTouch;

    ChestiiDeAutonom() {
    }

    ChestiiDeAutonom(boolean startThreads) {
        startThread = startThreads;
    }

    public void init(HardwareMap hard) {
        sliderL = hard.get(DcMotorEx.class, "sliderL");
        sliderR = hard.get(DcMotorEx.class, "sliderR");
        brat = hard.get(DcMotorEx.class, "brat");

        ghearaL = hard.get(Servo.class, "ghearaL");
        ghearaR = hard.get(Servo.class, "ghearaR");
        servodubios = hard.get(Servo.class, "servodubios");
        sliderTouch = hard.get(TouchSensor.class, "sliderTouch");

        sliderL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void target(int poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if(motor.getCurrentPosition() < poz){
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested()
                && lastTime + t > System.currentTimeMillis()
                && (Math.abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
    }

    public void slidertarget(int poz, double vel, double t, int tolerance) {
        Log.wtf("sliderL target:", Integer.toString(-poz));
        Log.wtf("sliderR target:", Integer.toString(poz));

        if(sliderR.getCurrentPosition() < poz){
            sliderL.setVelocity(-Math.abs(vel));
            sliderR.setVelocity(Math.abs(vel));
        }
        else{
            sliderL.setVelocity(Math.abs(vel));
            sliderR.setVelocity(-Math.abs(vel));
        }

        double lastTime = System.currentTimeMillis();
        Log.wtf("sliderL busy:",Double.toString(sliderL.getVelocity()));
        Log.wtf("sliderR busy:",Double.toString(sliderR.getVelocity()));
        while (Math.abs(poz - sliderR.getCurrentPosition()) > tolerance
                && Math.abs(poz - sliderL.getCurrentPosition()) > tolerance
                && !isStopRequested()
                && lastTime + t > System.currentTimeMillis()) {
            Log.wtf("sliderL:", Integer.toString(sliderL.getCurrentPosition()));
            Log.wtf("sliderR:", Integer.toString(sliderR.getCurrentPosition()));
        }
        sliderL.setVelocity(0);
        sliderR.setVelocity(0);
    }

    public void sliderHome(int vel, double t) {
        sliderL.setVelocity(vel);
        sliderR.setVelocity(-vel);
        double lastTime = System.currentTimeMillis();
        Log.wtf("sliderTouch",Boolean.toString(sliderTouch.isPressed()));
        while (!sliderTouch.isPressed()
                && !isStopRequested()
                && lastTime + t > System.currentTimeMillis()) {
                Log.wtf(String.valueOf(System.currentTimeMillis()), String.valueOf(sliderTouch.isPressed()));
        }
        sliderL.setVelocity(0);
        sliderR.setVelocity(0);
    }

    public void close() {
        ghearaL.setPosition(0.1);
        ghearaR.setPosition(0.9);
    }

    public void open() {
        ghearaL.setPosition(0.9);
        ghearaR.setPosition(0.1);
    }

    public void kdf(int t) {
        double lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()) ;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
