package org.firstinspires.ftc.teamcode

class Constants {
    object Drive {
        //TODO: THIS IS FOR DYLANS ROBOT, CHANGE FOR KAISER
        const val FRONT_LEFT = "frontLeft"
        const val FRONT_RIGHT = "backRight"
        const val BACK_LEFT = "backLeft"
        const val BACK_RIGHT = "frontRight"

        const val IMU = "navx"
    }

    object Vision {
        const val DESIRED_DISTANCE = 12.0;
        const val DESIRED_TAG_ID = 4;

        const val SPEED_GAIN =  0.03;
        const val STRAFE_GAIN =  0.015;
        const val TURN_GAIN =  0.04;

        const val MAX_AUTO_SPEED = 0.8;
        const val MAX_AUTO_STRAFE = 0.7;
        const val MAX_AUTO_TURN = 0.3;

        const val USE_WEBCAM = true
    }
}