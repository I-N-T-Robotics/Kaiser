package org.firstinspires.ftc.teamcode.Testing;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.Sharko;

@TeleOp(name = "Twyls Arm Test", group = "Twyls and Darius")
public class TywlsTest extends OpMode {

    private Servo pitchServo, rollServo, leftPivot, rightPivot, claw;
    private DcMotorEx LIFT_1, LIFT_2, intake;

    private PIDController controller;

    public static int target = 0;

    public enum armStates {
        START,
        UNDERPASS,
        COLLECT,
        STORE,
        LOW,
        MID,
        HIGH
    }

    Sharko.armStates state = Sharko.armStates.START;
    @Override
    public void init() {

        LIFT_1 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_1);
        LIFT_2 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_2);
        intake = hardwareMap.get(DcMotorEx.class, "intake roller");

        LIFT_1.setDirection(DcMotorSimple.Direction.REVERSE);

        LIFT_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchServo = hardwareMap.servo.get("pitch");
        rollServo = hardwareMap.servo.get("twist");

        leftPivot = hardwareMap.servo.get("left pivot");
        rightPivot = hardwareMap.servo.get("right pivot");

        //claw = hardwareMap.servo.get("claw");

        leftPivot.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rollServo.setPosition(0.5);
        pitchServo.setPosition(0.3);
       // claw.setPosition(0.5);

        controller = new PIDController(Constants.Lift.ARM_P, Constants.Lift.ARM_I, Constants.Lift.ARM_D);

        setPivot(0.33);
    }

    ElapsedTime eventTimer = new ElapsedTime();

    @Override
    public void loop() {
        controller.setPID(Constants.Lift.ARM_P, Constants.Lift.ARM_I, Constants.Lift.ARM_D);
        int pos = LIFT_1.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / Constants.Arm.ticksPer) * Constants.Lift.ARM_F);

        double power = pid + ff;

        LIFT_1.setPower(power);
        LIFT_2.setPower(power);

        if (gamepad1.right_bumper) {
          intake.setPower(0.6);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.dpad_down) {
          //  claw.setPosition(0.5);
        }

        switch (state) {

            case START:
                target = 80;

                setPivot(0.33);
                pitchServo.setPosition(0.54);


                if (gamepad2.left_bumper) {
                    eventTimer.reset();
                    state = Sharko.armStates.COLLECT;
                }

                break;

            case COLLECT:
                target = Constants.Arm.LIFT_START;

                setPivot(Constants.Arm.PIVOT_COLLECT);
                pitchServo.setPosition(Constants.Arm.PITCH_START);

                if (gamepad2.a) {
                  //  claw.setPosition(Constants.Arm.CLOSE);
                    eventTimer.reset();
                    state = Sharko.armStates.STORE;
                }

                break;

            case STORE:

                if (gamepad2.a) {
                    target = Constants.Arm.LIFT_START;

                    setPivot(Constants.Arm.PIVOT_STORE);
                    pitchServo.setPosition(Constants.Arm.PITCH_STORE);
                }

                if (gamepad2.y) {
                    eventTimer.reset();
                    state = Sharko.armStates.UNDERPASS;
                }

                break;

            case UNDERPASS:
                target = Constants.Arm.LIFT_MAX;

                if (gamepad2.y && (Math.abs(Constants.Arm.LIFT_MAX - pos) <= 15)) {
                    setPivot(Constants.Arm.PIVOT_MID);
                    pitchServo.setPosition(Constants.Arm.PITCH_MID);
                    state = Sharko.armStates.MID;
                }

                if (gamepad2.x && (Math.abs(Constants.Arm.LIFT_MAX - pos) <= 15)) {
                    setPivot(Constants.Arm.PIVOT_START);
                    pitchServo.setPosition(Constants.Arm.PITCH_START);

                    if (eventTimer.time() >= 3) {
                        state = Sharko.armStates.START;
                    }

                }

                break;

            case MID:
                if (gamepad2.y) {
                    target = Constants.Arm.LIFT_MID;
                }

                if (gamepad2.a) {
                   // claw.setPosition(Constants.Arm.OPEN);
                }

                if (gamepad2.x) {
                    eventTimer.reset();
                    state = Sharko.armStates.UNDERPASS;
                }

                break;

            default:
                state = Sharko.armStates.START;

        }}

    public void setPivot(double pos) {
        leftPivot.setPosition(pos);
        rightPivot.setPosition(pos);
    }
}
