package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
object Config {
    var behavior = Behavior.MANUAL
    var allianceColour = SampleColor.UNKNOWN

    var LINEAR_SLIDE_PIDF = PIDFCoefficients()
    var LIFT_PIDF = PIDFCoefficients()

    enum class Behavior {
        COLLECTING,
        MANUAL,
        RUN_TO_VISION_POSITION
    }

    enum class SampleColor {
        RED, BLUE, YELLOW, UNKNOWN
    }
}