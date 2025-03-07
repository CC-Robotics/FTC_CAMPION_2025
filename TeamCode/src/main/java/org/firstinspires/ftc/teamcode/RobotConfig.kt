package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
object RobotConfig {
    @JvmField var allianceColour = SampleColor.UNKNOWN

    @JvmField var LINEAR_SLIDE_PIDF = PIDFCoefficients(0.01, 0.002, 0.000001, 0.00005)
    @JvmField var LIFT_PIDF = PIDFCoefficients(0.012, 0.005, 0.0002, 0.002)

    @JvmField var VIKTEC_FF = 0.008

    @JvmField var lockLift = false
    @JvmField var lockServos = false

    @JvmField var startCollection = false

    enum class SampleColor {
        RED, BLUE, YELLOW, UNKNOWN
    }
}