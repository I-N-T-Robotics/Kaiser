package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Drive;

@Config
public class Robot {

    public DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public Robot(HardwareMap map) {
        frontLeftMotor = map.get(DcMotorEx.class, Drive.FRONT_LEFT);
        frontRightMotor = map.get(DcMotorEx.class, Drive.FRONT_RIGHT);
        backLeftMotor = map.get(DcMotorEx.class, Drive.BACK_LEFT);
        backRightMotor = map.get(DcMotorEx.class, Drive.BACK_RIGHT);

        //TODO: Change this to left and fix joystick inversion
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}

