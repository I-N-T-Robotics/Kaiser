package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Tram;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous(name = "Left Side Red", group = "Red Autos")
public class RedLeft extends OpMode {
    private VisionPortal visionPortal;
    private Tram redTram;
    private RevColorSensorV3 colorSensorV3;

    @Override
    public void init() {
        Scalar lower = new Scalar(150, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);

        double minArea = 100;

        redTram = new Tram(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(redTram)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", redTram.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + redTram.getLargestContourX() + ", y: " + redTram.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", redTram.getLargestContourArea());
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        Tram.PropPositions recordedPropPosition = redTram.getRecordedPropPosition();

        if (recordedPropPosition == Tram.PropPositions.UNFOUND) {
            recordedPropPosition = Tram.PropPositions.MIDDLE;
        }

        switch (recordedPropPosition) {
            case LEFT:

                break;
            case UNFOUND:
            case MIDDLE:

                break;
            case RIGHT:
                DriveTrain drive = new DriveTrain(hardwareMap, new Pose2d(11.58, -59.01, Math.toRadians(90.00)));


                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(23.25, -33.22), Math.toRadians(11.31))
                        .splineTo(new Vector2d(46.59, -36.80), Math.toRadians(0.00))
                        .splineTo(new Vector2d(35.11, -53.18), Math.toRadians(224.34))
                        .splineTo(new Vector2d(4.61, -59.39), Math.toRadians(180.00))
                        .splineTo(new Vector2d(-27.34, -60.26), Math.toRadians(180.00))
                        .splineTo(new Vector2d(-56.38, -38.87), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-44.93, -28.11), Math.toRadians(-4.22))
                        .splineTo(new Vector2d(-33.14, -53.96), Math.toRadians(-19.44))
                        .splineTo(new Vector2d(-13.19, -53.96), Math.toRadians(-4.76))
                        .splineTo(new Vector2d(49.27, -44.07), Math.toRadians(4.40))
                        .build());
                break;
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        redTram.close();
        visionPortal.close();
    }
}