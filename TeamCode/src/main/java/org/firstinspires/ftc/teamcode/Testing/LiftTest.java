package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "LiftTest" , group = "Dairy Queen")
@Config

public class LiftTest extends OpMode {

    private DcMotorEx LIFT_1;
    private DcMotorEx LIFT_2;

    private Servo leftPivot, rightPivot;

    public static double p = 0.09, i = 0.04, d = 0.0007225, f = 0.432;

    public static int target = 0;

    public static double tickPer = 145.1 / 180;

    private PIDController controller;

    @Override
    public void init() {
        LIFT_1 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_1);
        LIFT_2 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_2);

        LIFT_1.setDirection(DcMotorSimple.Direction.REVERSE);

        //leftPivot = hardwareMap.servo.get("left pivot");
        // rightPivot = hardwareMap.servo.get("right pivot");

        //leftPivot.setDirection(Servo.Direction.REVERSE);

        //setPivot(0.3, 0.3);


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        //TODO: Currently only works with one lift
        controller.setPID(p, i, d);
        int pos = LIFT_1.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / tickPer) * f);

        double power = pid + ff;

        LIFT_1.setPower(power);
        LIFT_2.setPower(power);


        telemetry.addData("current pos", pos);
        telemetry.addData("br", LIFT_2.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.update();

    }

    public void setPivot(double leftPos, double rightPos) {
        leftPivot.setPosition(leftPos);
        rightPivot.setPosition(rightPos);
    }
}


