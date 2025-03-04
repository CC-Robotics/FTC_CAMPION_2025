package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import dev.frozenmilk.dairy.core.FeatureRegistrar
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.roundToInt

object Util {
    @JvmField val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, FeatureRegistrar.activeOpMode.telemetry)
}

fun lerpPIDFValues(v1: PIDFCoefficients, v2: PIDFCoefficients, p: Double, places: Int = 3): PIDFCoefficients {
    return PIDFCoefficients(
        lerp(v1.p, v2.p, p).round(places),
        lerp(v1.i, v2.i, p).round(places),
        lerp(v1.d, v2.d, p).round(places),
        lerp(v1.f, v2.f, p).round(places)
    )
}

fun Double.round(decimals: Int): Double {
    val factor = 10.0.pow(decimals)
    return (this * factor).roundToInt() / factor
}

fun lerp(a: Double, b: Double, p: Double): Double {
    return a + (b - a) * p
}

fun lerp(a: Int, b: Int, p: Double): Double {
    return a + (b - a) * p
}

fun lerpRound(a: Int, b: Int, p: Double): Int {
    return (a + (b - a) * p).roundToInt()
}

fun applySensitivity(
    value: Double,
    upSensitivity: Double,
    downSensitivity: Double = upSensitivity
): Double {
    if (value == 0.0) return 0.0
    return if (value > 0) {
        value * upSensitivity
    } else {
        value * downSensitivity
    }
}

fun basically(a: Double, b: Double, epsilon: Double): Boolean {
    return abs(a - b) < epsilon
}

fun basically(a: Int, b: Int, epsilon: Int): Boolean {
    return abs(a - b) < epsilon
}

fun clampInt(x: Int, lo: Int, hi: Int): Int {
    if (x < lo) {
        return lo
    }
    if (x > hi) {
        return hi
    }
    return x
}