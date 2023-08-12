package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "drive")
public class AutonomStanga extends LinearOpMode {
    double rectx, recty, hperw;
    int varrez = 2;
    public OpenCvCamera webcam;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    ChestiiDeAutonom c = new ChestiiDeAutonom(true);

    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        c.close();
        while (!isStarted() && !isStopRequested()) {
            try {
                rectx = pipeline.getRect().width;
                recty = pipeline.getRect().height;
                hperw = recty / rectx;
                telemetry.addData("rectangle width:", rectx);
                telemetry.addData("rectangle height:", recty);
                telemetry.addData("height / width:", hperw);
                if (hperw < 0.9) {
                    varrez = 3;
                } else if (hperw < 3) {
                    varrez = 1;
                } else {
                    varrez = 2;
                }
                telemetry.addData("caz:", varrez);
            } catch (Exception E) {
                telemetry.addData("Webcam error:", "please restart");
            }
            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(32.08, 62.79, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        waitForStart();
        c.target(100, 500, c.brat, 10000, 10);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35.08, 58))
                .turn(Math.toRadians(180))
                .splineTo(new Vector2d(35.08, 18), Math.toRadians(270))
                .splineTo(new Vector2d(35.08, 11), Math.toRadians(214))
//                                .lineToLinearHeading(new Pose2d(35.8, 10, Math.toRadians(135)))
                .build();
        drive.followTrajectorySequence(ts);
        c.target(1700, 2000, c.brat, 10000, 10);
        c.slidertarget(1550, 2000, 3000, 10);
        c.slidertarget(1200, 2000, 10000, 10);
//        c.kdf(1000);
        c.open();
        c.kdf(250);
        c.target(900, 2000, c.brat, 10000, 10);
        c.sliderHome(2000, 2000);
        c.target(0, 2000, c.brat, 10000, 10);

        switch (varrez) {
            case 1:
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36, 13, Math.toRadians(180)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(54,13,Math.toRadians(180)))
                        .turn(Math.toRadians(90))
                        .back(3)
                        .build();
                drive.followTrajectorySequence(ts);
                break;
            case 2:
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36, 13, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(32, 13, Math.toRadians(270)))
                        .back(3)
                        .build();
                drive.followTrajectorySequence(ts);
                break;
            case 3:
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36, 11, Math.toRadians(180)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(8,11,Math.toRadians(180)))
                        .turn(Math.toRadians(90))
                        .back(3)
                        .build();
                drive.followTrajectorySequence(ts);
                break;
        }

        while (!isStopRequested()) {
            telemetry.addData("sliderL:", c.sliderL.getCurrentPosition());
            telemetry.addData("sliderR:", c.sliderL.getCurrentPosition());
            telemetry.update();
        }
    }
}
