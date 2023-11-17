package org.firstinspires.ftc.teamcode


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

        const val ARM_P = 0.09
        const val ARM_I = 0.04
        const val ARM_D = 0.0007225
        const val ARM_F = 0.432
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

    object Arm {
        const val ticksPer = 145.1 / 180

        //todo FIND ALL THESE FUCKING VALUES RAHH
        const val PIVOT_START = 0.3
        const val PIVOT_COLLECT = 0.3
        const val PIVOT_STORE = 0.3
        const val PIVOT_MID = 0.3

        const val PITCH_START = 0.5
        const val PITCH_STORE = 0.5;
        const val PITCH_MID = 0.5;
        const val PITCH_COLLECT = 0.5;

        const val OPEN = 0.5
        const val CLOSE = 0.5;

        const val LIFT_START = 20
        const val LIFT_MID = 20
        const val LIFT_MAX = 550
    }
}