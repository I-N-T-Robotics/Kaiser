package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RoadRunner.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Tram;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Right Side Blue", group = "Blue Autos")
public class BlueRight extends OpMode {
    private VisionPortal visionPortal;
    private Tram blueTram;
    @Override
    public void init() {
        Scalar lower = new Scalar(100,150,0);
        Scalar upper = new Scalar(140,255,255);
        double minArea = 100;

        blueTram = new Tram(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blueTram)
                .build();

        try {
            setManualExposure(6, 250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {

        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                Thread.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);


    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", blueTram.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + blueTram.getLargestContourX() + ", y: " + blueTram.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", blueTram.getLargestContourArea());
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        Tram.PropPositions recordedPropPosition = blueTram.getRecordedPropPosition();

        if (recordedPropPosition == Tram.PropPositions.UNFOUND) {
            recordedPropPosition = Tram.PropPositions.MIDDLE;
        }

        switch (recordedPropPosition) {
            case LEFT:

                break;
            case UNFOUND:
            case MIDDLE:
                DriveTrain drive = new DriveTrain(hardwareMap, new Pose2d(11.76, 60.33, Math.toRadians(-90.00)));

                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .lineToY(33.04)
                        .lineToY(43.76)
                        .splineTo(new Vector2d(27.41, 37.13), Math.toRadians(4.91))
                        .splineToSplineHeading(new Pose2d(49.97, 36.78, Math.toRadians(-1.00)), Math.toRadians(-11.31))
                        .build());

                break;
            case RIGHT:

                break;
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        blueTram.close();
        visionPortal.close();
    }
}
