package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.DriveTrain;

@Autonomous (name = "Park")
public class Park extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap, new Pose2d(0, 0, 0));

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build());

    }
}
