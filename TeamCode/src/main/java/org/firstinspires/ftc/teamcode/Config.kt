package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
object Config {
    @JvmField var behavior = Behavior.MANUAL
    @JvmField var allianceColour = SampleColor.UNKNOWN

    @JvmField var LINEAR_SLIDE_PIDF = PIDFCoefficients()
    @JvmField var LIFT_PIDF = PIDFCoefficients()

    enum class Behavior {
        COLLECTING,
        MANUAL,
        RUN_TO_VISION_POSITION
    }

    enum class SampleColor {
        RED, BLUE, YELLOW, UNKNOWN
    }
}