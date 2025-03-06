package org.firstinspires.ftc.teamcode.controller

class PController(private var kp: Double) {
    fun calculate(mp: Double, sp: Double): Double {
        val error = sp - mp
        return kp * error
    }
}
