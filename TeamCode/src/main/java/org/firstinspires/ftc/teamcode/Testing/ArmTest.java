package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name = "Arm testing", group = "TeleOp Tests")
public class ArmTest extends OpMode {

    public DcMotorEx LIFT_1, LIFT_2;

    @Override
    public void init() {
        LIFT_1 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_1);
        LIFT_2 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_2);

        LIFT_2.setDirection(DcMotorSimple.Direction.REVERSE);

        LIFT_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        //TODO get the right inversions
        double slidePower = gamepad1.left_stick_y;

        LIFT_1.setPower(slidePower);
        LIFT_2.setPower(slidePower);

        telemetry.addData("LIFT 1 TICKS:", LIFT_1.getCurrentPosition());
        telemetry.addData("LIFT 2 TICKS:", LIFT_2.getCurrentPosition());
    }
}
