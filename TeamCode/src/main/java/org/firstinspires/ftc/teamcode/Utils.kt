package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.controller.PIDFValues
import kotlin.math.pow
import kotlin.math.roundToInt

fun lerpPIDFValues(v1: PIDFValues, v2: PIDFValues, p: Double): PIDFValues {
    return PIDFValues(
        lerpf(v1.p, v2.p, p),
        lerpf(v1.i, v2.i, p),
        lerpf(v1.d, v2.d, p),
        lerpf(v1.f, v2.f, p)
    )
}

fun lerpPIDFValuesRounded(v1: PIDFValues, v2: PIDFValues, p: Double, places: Int = 3): PIDFValues {
    return PIDFValues(
        lerpf(v1.p, v2.p, p).round(places),
        lerpf(v1.i, v2.i, p).round(places),
        lerpf(v1.d, v2.d, p).round(places),
        lerpf(v1.f, v2.f, p).round(places)
    )
}

fun Double.round(decimals: Int): Double {
    val factor = 10.0.pow(decimals)
    return (this * factor).roundToInt() / factor
}

fun lerpf(a: Double, b: Double, p: Double): Double {
    return a + (b - a) * p
}

fun applySensitivity(value: Double, upSensitivity: Double, downSensitivity: Double = upSensitivity): Double {
    if (value == 0.0) return 0.0
    val sensitivity = if (value > 0) upSensitivity else downSensitivity
    return value.pow(sensitivity)
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