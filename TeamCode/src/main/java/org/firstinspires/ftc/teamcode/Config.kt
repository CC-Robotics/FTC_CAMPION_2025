package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
object Config {
    @JvmField var behavior = Behavior.MANUAL
    @JvmField var allianceColour = SampleColor.UNKNOWN

    @JvmField var LINEAR_SLIDE_PIDF = PIDFCoefficients(0.02, 0.002, 0.000001, 0.00005)
    @JvmField var LIFT_PIDF = PIDFCoefficients(0.01, 0.002, 0.0002, 0.0006)

    enum class Behavior {
        COLLECTING,
        MANUAL,
        RUN_TO_VISION_POSITION
    }

    enum class SampleColor {
        RED, BLUE, YELLOW, UNKNOWN
    }
}