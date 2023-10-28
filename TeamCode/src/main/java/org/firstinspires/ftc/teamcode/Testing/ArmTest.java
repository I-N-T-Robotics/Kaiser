package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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

    private PIDController controller;
    private PIDController armController;

    private double setPosition = -530;
    private double startArmL = 367;
    private double startArmR = -127;
    private double lowArmL = 119;
    private double lowArmR = -370;

    public static int target = 0;
    public static double p = 0, i = 0, d = 0;
    public static int f = 0;

    public final double tickPer = 145.1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LIFT_1 = hardwareMap.get(DcMotorEx.class, Lift.LIFT_1);
        LIFT_2 = hardwareMap.get(DcMotorEx.class, Lift.LIFT_2);

        LIFT_2.setDirection(DcMotorSimple.Direction.REVERSE);
        LIFT_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ARM_1 = hardwareMap.servo.get("ARM 1");
        ARM_2 = hardwareMap.servo.get("ARM 2");
        pivot = hardwareMap.servo.get("pivot");

        ARM_1.setPosition(0);
        ARM_2.setPosition(0);

        ARM_2.setDirection(Servo.Direction.REVERSE);

        controller = new PIDController(p, i, d);

    }

    @Override
    public void loop() {

        controller.setPID(p, i, d);
        int liftPosition = LIFT_1.getCurrentPosition();
        double output = controller.calculate(liftPosition, target);
        double ff = Math.cos(Math.toRadians(target / tickPer) * f);

        double power = output + ff;

        LIFT_1.setPower(power);
        // LIFT_2.setPower(power);


//        double leftArmPos = armEncoderLeft.getVoltage() / 3.3 * 360;
//        double rightArmPos = armEncoderRight.getVoltage() / 3.3 * 360;

       /*  double outputTop = controller.calculate(liftPosition, -400);
        double outputGround = controller.calculate(liftPosition, -100);

        if (gamepad1.dpad_up) {
            target = -400;
            // LIFT_2.setVelocity(outputTop);
        }
        if (gamepad1.dpad_down) {
            target = -100;
        }

        if (gamepad1.a) {
            ARM_2.setPosition(0.6);
            ARM_1.setPosition(0.6);
        } else if (gamepad1.b) {
            ARM_1.setPosition(0);
            ARM_2.setPosition(0);
        } */


        telemetry.addData("LIFT 1 TICKS:", LIFT_1.getCurrentPosition());
        telemetry.addData("LIFT 2 TICKS:", LIFT_2.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.update();

    }
}
