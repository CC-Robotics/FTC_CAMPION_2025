package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
object Config {
    @JvmField var behavior = Behavior.MANUAL
    @JvmField var allianceColour = SampleColor.UNKNOWN

    @JvmField var LINEAR_SLIDE_PIDF = PIDFCoefficients()
    @JvmField var LIFT_PIDF = PIDFCoefficients(0.027, 0.0027, 0.00001, 0.00007)

    enum class Behavior {
        COLLECTING,
        MANUAL,
        RUN_TO_VISION_POSITION
    }

    enum class SampleColor {
        RED, BLUE, YELLOW, UNKNOWN
    }
}