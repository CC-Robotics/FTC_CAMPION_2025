package org.firstinspires.ftc.teamcode.subsystem.outdated

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.Config
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import java.lang.annotation.Inherited

/**
 * Lift RithVIK TEChnology Subsystem:
 * Uses raw power and switches to feedforward control to keep lift in place
 * */
object LiftViktecSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Lift"

    private val rightLift by subsystemCell { getHardware<DcMotorEx>("right_lift") }
    private val leftLift by subsystemCell { getHardware<DcMotorEx>("left_lift") }

    // private const val MAX_POSITION = 1500
    private const val POWER = 0.5

    private lateinit var dashboardTelemetry: MultipleTelemetry

    override fun init(opMode: Wrapper) {
        leftLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        rightLift.direction = DcMotorSimple.Direction.REVERSE

        leftLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun update(upButton: Boolean, downButton: Boolean) {
        val multiplier = if (upButton && !downButton) {
            1.0
        } else if (!upButton && downButton) {
            -1.0
        } else {
            0.0
        }

        leftLift.power = Config.VIKTEC_FF + POWER * multiplier
        rightLift.power = Config.VIKTEC_FF + POWER * multiplier

        telemetry()
    }

    fun telemetry() {
        dashboardTelemetry.addData("Left Lift Position", leftLift.currentPosition)
        dashboardTelemetry.addData("Right Lift Position", rightLift.currentPosition)

        dashboardTelemetry.addData("Left Power", leftLift.power)
        dashboardTelemetry.addData("Right Power", rightLift.power)

        dashboardTelemetry.update()

        telemetry.addData("Left Lift Real Position", leftLift.currentPosition)
        telemetry.addData("Right Lift Real Position", rightLift.currentPosition)
        telemetry.addData("Left Lift Power (real vs intended)", "${leftLift.power}")
        telemetry.addData("Right Lift Power (real vs intended)", "${rightLift.power}")
    }
}