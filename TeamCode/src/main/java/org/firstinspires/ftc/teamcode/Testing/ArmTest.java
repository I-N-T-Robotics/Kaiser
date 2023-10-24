package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.hardware.usb.ArmableUsbDevice;
import org.firstinspires.ftc.teamcode.Constants.Lift;

@Config
@TeleOp(name = "Arm testing", group = "TeleOp Tests")
public class ArmTest extends OpMode {

    public DcMotorEx LIFT_1, LIFT_2;
    public Servo ARM_1, ARM_2, pivot;
    public AnalogInput armEncoderLeft, armEncoderRight, pivotEncoder;

    private PIDController controller;
    private PIDController armController;

    private double setPosition = -530;
    private double startArmL = 367;
    private double startArmR = -127;
    private double lowArmL = 119;
    private double lowArmR = -370;

    int target = -50;

    @Override
    public void init() {
        LIFT_1 = hardwareMap.get(DcMotorEx.class, Lift.LIFT_1);
        LIFT_2 = hardwareMap.get(DcMotorEx.class, Lift.LIFT_2);

        LIFT_2.setDirection(DcMotorSimple.Direction.REVERSE);

        LIFT_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        LIFT_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LIFT_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LIFT_1.setPositionPIDFCoefficients(3.5);
        LIFT_2.setPositionPIDFCoefficients(3.5);
        LIFT_1.setTargetPositionTolerance(10);
        LIFT_2.setTargetPositionTolerance(10);


        ARM_1 = hardwareMap.servo.get("ARM 1");
        ARM_2 = hardwareMap.servo.get("ARM 2");
        pivot = hardwareMap.servo.get("pivot");

        ARM_1.setPosition(0.5);
        ARM_2.setPosition(0.5);

        armEncoderLeft = hardwareMap.get(AnalogInput.class, "leftArmEncoder");
        armEncoderRight = hardwareMap.get(AnalogInput.class, "rightArmEncoder");


        controller = new PIDController(Lift.ARM_P, Lift.ARM_I, Lift.ARM_D);
        controller.setSetPoint(setPosition);

        armController = new PIDController(0.1, 0, 0);
    }

    @Override
    public void loop() {
        LIFT_1.setTargetPosition(target);
        LIFT_2.setTargetPosition(target);
        LIFT_1.setPower(1);
        LIFT_2.setPower(1);
        LIFT_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LIFT_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double  liftPosition = LIFT_1.getCurrentPosition();
        double output = controller.calculate(liftPosition, setPosition);

        double leftArmPos = armEncoderLeft.getVoltage() / 3.3 * 360;
        double rightArmPos = armEncoderRight.getVoltage() / 3.3 * 360;

        double outputTop = controller.calculate(liftPosition, -500);
        double outputGround = controller.calculate(liftPosition, -50);

        if (gamepad1.dpad_up) {
            target = -500;
        }

        if (gamepad1.dpad_down) {
            target = -50;
        }





        telemetry.addData("LIFT 1 TICKS:", LIFT_1.getCurrentPosition());
        telemetry.addData("LIFT 2 TICKS:", LIFT_2.getCurrentPosition());
        telemetry.addData("ARM 1 POS", armEncoderLeft.getVoltage() / 3.3 * 360);
        telemetry.addData("ARM 2 POS", (armEncoderRight.getVoltage() / 3.3 * 360));
       // telemetry.addData("pivot pos", pivotEncoder.getVoltage() / 3.3 * 360);
    }
}
