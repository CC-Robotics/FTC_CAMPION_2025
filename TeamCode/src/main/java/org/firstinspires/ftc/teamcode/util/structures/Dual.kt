package org.firstinspires.ftc.teamcode.util.structures

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx

class Dual(private var motor1: DcMotorEx, private var motor2: DcMotorEx) {
    var power: Double
        get() = motor1.power
        set(value) {
            motor1.power = value
            motor2.power = value
        }

    var mode: RunMode
        get() = motor1.mode
        set(value) {
            motor1.mode = value
            motor2.mode = value
        }

    fun reset() {
        mode = RunMode.STOP_AND_RESET_ENCODER
        mode = RunMode.RUN_WITHOUT_ENCODER
    }
}