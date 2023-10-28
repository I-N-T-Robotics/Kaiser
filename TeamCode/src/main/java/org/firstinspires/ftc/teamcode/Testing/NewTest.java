package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
@Config
public class NewTest extends OpMode {
    private PIDController controller;

    public static double p = 0.09, i = 0.04, d = 0.0007225, f = 0.432;

    public static int target = 0;

    public static double tickPer = 145.1 / 180;

    private DcMotorEx LIFT_1;
    private DcMotorEx LIFT_2;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        LIFT_1 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_1);
        LIFT_1.setDirection(DcMotorSimple.Direction.REVERSE);

        LIFT_2 = hardwareMap.get(DcMotorEx.class, Constants.Lift.LIFT_2);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int pos = LIFT_2.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / tickPer) * f);

        double power = pid + ff;

        // LIFT_1.setPower(power);
        LIFT_2.setPower(power);

        telemetry.addData("current pos", pos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}


