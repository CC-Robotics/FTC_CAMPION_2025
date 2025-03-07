package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.outdated.FieldCentricDrivetrainSubsystem
import kotlin.math.abs

@FieldCentricDrivetrainSubsystem.Attach
@Autonomous(name = "N/A | Auto - N/A | Simple Main", group = "2024-25 OpModes")
class AutoSimple : OpMode() {
    companion object {
        const val COUNTS_PER_MOTOR_REV: Double = 537.7
        const val DRIVE_GEAR_REDUCTION: Double = 1.0
        const val WHEEL_DIAMETER_INCHES: Double = 3.8
        const val COUNTS_PER_INCH: Double = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI)
        const val DRIVE_SPEED: Double = 0.6
        const val TURN_SPEED: Double = 0.5
    }

    override fun init() {
        TODO("Not yet implemented")
    }

    override fun loop() {
        TODO("Not yet implemented")
    }
}