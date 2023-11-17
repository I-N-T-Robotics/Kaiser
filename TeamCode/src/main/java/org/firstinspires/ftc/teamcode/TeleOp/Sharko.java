package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.github.i_n_t_robotics.zhonyas.navx.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Constants.Lift;
import org.firstinspires.ftc.teamcode.Constants.Drive;
import org.firstinspires.ftc.teamcode.Constants.Vision;
import org.firstinspires.ftc.teamcode.RoadRunner.DriveTrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name= "sussy")
public class Sharko extends OpMode {

    // APRIL TAG
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    // DRIVETRAIN MOTOR DECLARATIONS
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public Servo ARM_1, ARM_2;

    // MITSUMI MOTOR AND SERVO DECLARE
    public DcMotorEx LIFT_1, LIFT_2;

    // NAVX DECLARATION
    private AHRS imu;

    private PIDController controller;

    private double setPositionTop = -530;

    public enum armStates {
        START,
        UNDERPASS,
        COLLECT,
        SCORE,
        PLACEMID
    }

    armStates state = armStates.START;

    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get(Drive.FRONT_LEFT);
        backLeftMotor = hardwareMap.dcMotor.get(Drive.BACK_LEFT);
        frontRightMotor = hardwareMap.dcMotor.get(Drive.FRONT_RIGHT);
        backRightMotor = hardwareMap.dcMotor.get(Drive.BACK_RIGHT);


        LIFT_1 = hardwareMap.get(DcMotorEx.class, Lift.LIFT_1);
        LIFT_2 = hardwareMap.get(DcMotorEx.class, Lift.LIFT_2);

        //TODO: Change this to left and fix joystick inversion
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LIFT_2.setDirection(DcMotorSimple.Direction.REVERSE);

        LIFT_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LIFT_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ARM_1 = hardwareMap.servo.get("ARM 1");
        ARM_2 = hardwareMap.servo.get("ARM 2");

        ARM_2.setDirection(Servo.Direction.REVERSE);

        ARM_1.setPosition(0.2);
        ARM_2.setPosition(0.2);

        imu = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        controller = new PIDController(Lift.ARM_P, Lift.ARM_I, Lift.ARM_D);
        DriveTrain drive = new DriveTrain(hardwareMap, new Pose2d(0, 0, 0));


        initAprilTag();

        if (Vision.USE_WEBCAM) {
            try {
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

    }

    boolean targetFound = false;
    double drive = 0;
    double strafe = 0;
    double turn = 0;

    public void loop() {

        if (gamepad1.a) {
            ARM_1.setPosition(0.2);
            ARM_2.setPosition(0.2);
        }

        if (gamepad1.b) {
            ARM_1.setPosition(0.4);
            ARM_2.setPosition(0.4);
        }

        if (gamepad1.x) {
            ARM_1.setPosition(0.8);
            ARM_2.setPosition(0.8);
        }

        if (gamepad1.y) {
            ARM_1.setPosition(0.7);
            ARM_2.setPosition(0.7);
        }

        switch (state) {

            case START:

        }

        ///////////////// APRILTAG SETUP /////////////////
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((Vision.DESIRED_TAG_ID < 0) || (detection.id == Vision.DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        ///////////////// FCD SETUP /////////////////
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.zeroYaw();
        }

        double botHeading = -Math.toRadians(imu.getYaw());

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.left_bumper && targetFound) {
            double  rangeError      = (desiredTag.ftcPose.range - Vision.DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            drive  = Range.clip(-rangeError * Vision.SPEED_GAIN, -Vision.MAX_AUTO_SPEED, Vision.MAX_AUTO_SPEED);
            turn   = Range.clip(-headingError * Vision.TURN_GAIN, -Vision.MAX_AUTO_TURN, Vision.MAX_AUTO_TURN) ;
            strafe = Range.clip(yawError * Vision.STRAFE_GAIN, -Vision.MAX_AUTO_STRAFE, Vision.MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            double leftFrontPower =  drive - strafe - turn;
            double rightFrontPower =  drive + strafe + turn;
            double leftBackPower  =  drive + strafe - turn;
            double rightBackPower =  drive - strafe + turn;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            frontLeftMotor.setPower(leftFrontPower);
            frontRightMotor.setPower(rightFrontPower);
            backLeftMotor.setPower(leftBackPower);
            backRightMotor.setPower(rightBackPower);
        }

        telemetry.update();

    }


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (Vision.USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
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
        //TODO: Worthless cast, try removing
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}

