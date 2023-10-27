package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;

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

    public Motor LIFT_1, LIFT_2;
    public Servo ARM_1, ARM_2, pivot;

    private PIDController controller;
    private PIDController armController;

    private double setPosition = -530;
    private double startArmL = 367;
    private double startArmR = -127;
    private double lowArmL = 119;
    private double lowArmR = -370;

    int target = 0;

    @Override
    public void init() {
        LIFT_1 = new Motor(hardwareMap, Lift.LIFT_1);
        LIFT_2 = new Motor(hardwareMap, Lift.LIFT_2);

        LIFT_2.setInverted(true);

        LIFT_1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        LIFT_2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        ARM_1 = hardwareMap.servo.get("ARM 1");
        ARM_2 = hardwareMap.servo.get("ARM 2");
        pivot = hardwareMap.servo.get("pivot");

        ARM_1.setPosition(0);
        ARM_2.setPosition(0);

        ARM_2.setDirection(Servo.Direction.REVERSE);

        controller = new PIDController(Lift.ARM_P, Lift.ARM_I, Lift.ARM_D);
        controller.setSetPoint(-400);

        armController = new PIDController(0.1, 0, 0);

        LIFT_1.setPositionCoefficient(0.1);
        LIFT_1.setTargetPosition(-400);
        LIFT_1.setPositionTolerance(10);

    }

    @Override
    public void loop() {

        LIFT_1.setRunMode(Motor.RunMode.PositionControl);

        double  liftPosition = LIFT_1.getCurrentPosition();
        double output = controller.calculate(liftPosition, setPosition);

//        double leftArmPos = armEncoderLeft.getVoltage() / 3.3 * 360;
//        double rightArmPos = armEncoderRight.getVoltage() / 3.3 * 360;

        double outputTop = controller.calculate(liftPosition, -400);
        double outputGround = controller.calculate(liftPosition, -100);

        if (gamepad1.dpad_up) {
       LIFT_1.set(0.5);
           // LIFT_2.setVelocity(outputTop);
        }
        if (gamepad1.dpad_down) {
          //  LIFT_2.setVelocity(outputGround);
        }

        if (gamepad1.a) {
            ARM_2.setPosition(0.6);
            ARM_1.setPosition(0.6);
        } else if (gamepad1.b) {
            ARM_1.setPosition(0);
            ARM_2.setPosition(0);
        }


        telemetry.addData("LIFT 1 TICKS:", LIFT_1.getCurrentPosition());
        telemetry.addData("LIFT 2 TICKS:", LIFT_2.getCurrentPosition());

    }
}
