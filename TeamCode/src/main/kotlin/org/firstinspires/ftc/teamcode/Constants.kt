package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.MecanumKinematics

object Constants {
    object Drive {
        //TODO: THIS IS FOR DYLANS ROBOT, CHANGE FOR KAISER
        const val FRONT_LEFT = "frontLeft"
        const val FRONT_RIGHT = "backRight"
        const val BACK_LEFT = "backLeft"
        const val BACK_RIGHT = "frontRight"

        const val IMU = "navx"

        object PARAMS {
            // Drive Model
            const val inPerTick = 0.000753;
            const val lateralInPerTick = -0.0005325981543257096;
            const val trackWidthTicks = 17375.305;

            // FF Values (ticks)
            const val kS = 0.8667242003254163;
            const val kV = 0.00015132445220638116;
            const val kA = 0.00005;

            //TODO See if these are even needed ngl
            const val maxWheelVel = 50;
            const val minProfileAccel = -30;
            const val maxProfileAccel = 50;

            const val maxAngVel = Math.PI; // shared with path
            const val maxAngAccel = Math.PI;

            const val axialGain = 6.0;
            const val lateralGain = 4.0;
            const val headingGain = 10.0; // shared with turn
            const val axialVelGain = 0.0;
            const val lateralVelGain = 0.0;
            const val headingVelGain = 0.0; // shared with turn
        }
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