package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Twyls Arm Test", group = "Twyls and Darius")
public class TywlsTest extends OpMode {

    private Servo pitchServo, rollServo, leftPivot, rightPivot;
    @Override
    public void init() {
        pitchServo = hardwareMap.servo.get("pitch");
        rollServo = hardwareMap.servo.get("roll");

        leftPivot = hardwareMap.servo.get("left pivot");
        rightPivot = hardwareMap.servo.get("right pivot");

        leftPivot.setDirection(Servo.Direction.REVERSE);

        rollServo.setPosition(0.5);
        pitchServo.setPosition(0.5);

        // setPivot(0.5, 0.5);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
          // setPivot(0.7, 0.7);
           // pitchServo.setPosition(0.7);
            rollServo.setPosition(0.7);
        }

        if (gamepad1.b) {
           // setPivot(0.3, 0.3);
            // pitchServo.setPosition(0.3);
            rollServo.setPosition(0.3);
        }

        if (gamepad1.x) {
           // setPivot(0.4, 0.4);
          //  pitchServo.setPosition(0.4);
            rollServo.setPosition(0.4);
        }

        if (gamepad1.y) {
           // setPivot(0.6, 0.6);
            // pitchServo.setPosition(0.6);
            rollServo.setPosition(0.6);
        }

    }

    public void setPivot(double leftPos, double rightPos) {
        leftPivot.setPosition(leftPos);
        rightPivot.setPosition(rightPos);
    }
}
