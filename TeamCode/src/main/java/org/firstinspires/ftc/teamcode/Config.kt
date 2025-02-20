package org.firstinspires.ftc.teamcode

object Config {
    var behavior = Behavior.MANUAL
    var allianceColour = SampleColor.UNKNOWN

    enum class Behavior {
        COLLECTING,
        MANUAL,
        RUN_TO_VISION_POSITION
    }

    enum class SampleColor {
        RED, BLUE, YELLOW, UNKNOWN
    }
}