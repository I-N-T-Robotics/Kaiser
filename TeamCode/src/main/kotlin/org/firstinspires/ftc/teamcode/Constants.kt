package org.firstinspires.ftc.teamcode

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