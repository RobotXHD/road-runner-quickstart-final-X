package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.BatteryChecker;

@Autonomous
public class AutonomDeBalta extends LinearOpMode {
    public DcMotorEx motorBL,motorBR,motorFL,motorFR;
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    ChestiiDeAutonom c = new ChestiiDeAutonom();
    public void Translatare(int time, double speed)
    {
        double lastTime = System.currentTimeMillis();
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        while(lastTime + time > System.currentTimeMillis()){

        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        if(!isStopRequested()) {
            c.close();
            c.kdf(750);
            c.target(100, 500, c.brat, 10000, 10);
            Translatare(10000, -0.1);
        }
    }

}
