package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name = "Arm testing", group = "TeleOp Tests")
public class ArmTest extends OpMode {

    public DcMotorEx LIFT_1, LIFT_2;

    @Override
    public void init() {
        LIFT_1 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_1);
        LIFT_2 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_2);
    }

    @Override
    public void loop() {
        //TODO get the right inversions
        double slidePower = gamepad1.left_stick_y;

        LIFT_1.setPower(slidePower);
        LIFT_2.setPower(slidePower);
    }
}
