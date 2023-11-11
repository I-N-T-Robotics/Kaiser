package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.ftc.DriveType
import com.acmerobotics.roadrunner.ftc.DriveView
import com.acmerobotics.roadrunner.ftc.Encoder
import com.github.i_n_t_robotics.zhonyas.navx.AHRS
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import java.util.Arrays

object Constants {
    object Drive {
        //TODO: THIS IS FOR DYLANS ROBOT, CHANGE FOR KAISER
        const val FRONT_LEFT = "frontLeft"
        const val FRONT_RIGHT = "frontRight"
        const val BACK_LEFT = "backLeft"
        const val BACK_RIGHT = "backRight"

        const val IMU = "navx"


        class TDWParams {
            @JvmField var par0YTicks: Double = -7883.0 // y position of the first parallel encoder (in tick units)
            @JvmField var par1YTicks: Double = 7999.1 // y position of the second parallel encoder (in tick units)
            @JvmField var perpXTicks: Double = 7387.25 // x position of the perpendicular encoder (in tick units)

        }
    }

    object Lift {
        const val LIFT_1 = "Lift 1"
        const val LIFT_2 = "Lift 2"

        const val ARM_P = 0.1
        const val ARM_I = 0
        const val ARM_D = 0.08
    }
    object Vision {
        const val DESIRED_DISTANCE = 12.0
        const val DESIRED_TAG_ID = 4

        const val SPEED_GAIN =  0.03
        const val STRAFE_GAIN =  0.015
        const val TURN_GAIN =  0.04

        const val MAX_AUTO_SPEED = 0.8
        const val MAX_AUTO_STRAFE = 0.7
        const val MAX_AUTO_TURN = 0.3

        const val USE_WEBCAM = true

    }

    class DriveViewer(
            val type: DriveType,
            val inPerTick: Double,
            val maxVel: Double,
            val minAccel: Double,
            val maxAccel: Double,
            val lynxModules: List<LynxModule>,
            // ordered front to rear
            val leftMotors: List<DcMotorEx>,
            val rightMotors: List<DcMotorEx>,
            // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
            //                  (parEncs.isEmpty() && perpEncs.isEmpty())
            leftEncs: List<Encoder>,
            rightEncs: List<Encoder>,
            parEncs: List<Encoder>,
            perpEncs: List<Encoder>,
            val imu: AHRS,
            val voltageSensor: VoltageSensor,
            val feedforward: MotorFeedforward,
    )

    interface DriveViewFactoryZ {
        fun make(h: HardwareMap): DriveViewer
    }
}