package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.TurnConstraints
import java.util.Arrays

object Constants {
    object Drive {
        //TODO: THIS IS FOR DYLANS ROBOT, CHANGE FOR KAISER
        const val FRONT_LEFT = "frontLeft"
        const val FRONT_RIGHT = "backRight"
        const val BACK_LEFT = "backLeft"
        const val BACK_RIGHT = "frontRight"

        const val IMU = "navx"

        class PARAMS {
            companion object {
                // Drive Model
                @JvmField var inPerTick: Double = 0.000753
                @JvmField var lateralInPerTick: Double = -0.0005325981543257096
                @JvmField var trackWidthTicks: Double = 17375.305
                
                // FF Values (ticks)
                @JvmField var kS: Double = 0.8667242003254163
                @JvmField var kV: Double = 0.00015132445220638116
                @JvmField var kA: Double = 0.00005
                
                // Path params (inches)
                @JvmField var maxWheelVel: Double = 50.0
                @JvmField var minProfileAccel: Double = -30.0
                @JvmField var maxProfileAccel: Double = 50.0
                
                // Turn profile parameters (in radians)
                @JvmField var maxAngVel: Double = Math.PI // shared with path
                @JvmField var maxAngAccel: Double = Math.PI

                // Path PID Values
                @JvmField var axialGain: Double = 6.0
                @JvmField var lateralGain: Double = 4.0
                @JvmField var headingGain: Double = 10.0 // shared with turn
                @JvmField var axialVelGain: Double = 0.0
                @JvmField var lateralVelGain: Double = 0.0
                @JvmField var headingVelGain: Double = 0.0 // shared with turn
            }
            
        }

        class TDWParams {
            @JvmField var par0YTicks: Double = -7883.0 // y position of the first parallel encoder (in tick units)
            @JvmField var par1YTicks: Double = 7999.1 // y position of the second parallel encoder (in tick units)
            @JvmField var perpXTicks: Double = 7387.25 // x position of the perpendicular encoder (in tick units)

        }


        @JvmField val kinematics = MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick)

        @JvmField val feedforward = MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

        @JvmField val defaultTurnConstraints = TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
        //TODO might also be a problem
        @JvmField val defaultVelConstraint = MinVelConstraint(Arrays.asList(
                kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
                AngularVelConstraint(PARAMS.maxAngVel)
        ))
        @JvmField val defaultAccelConstraint = ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
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
}