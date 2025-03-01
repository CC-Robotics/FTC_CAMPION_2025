package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.utils.applySensitivity
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import java.lang.annotation.Inherited

object LiftRTPSubsystem : PIDFSubsystem() {
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

    private const val MAX_POSITION = 1500

    override fun init(opMode: Wrapper) {
        targetPosition = 0

        leftLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        rightLift.direction = DcMotorSimple.Direction.REVERSE

        leftLift.targetPosition = 0
        rightLift.targetPosition = 0

        leftLift.power = 1.0
        rightLift.power = 1.0

        leftLift.targetPositionTolerance = 25
        rightLift.targetPositionTolerance = 25

        leftLift.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightLift.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun update(increment: Double) {
        changePosition(
            applySensitivity(increment, 1.0, 0.2)
        )
        clampPosition(0, MAX_POSITION)
        leftLift.targetPosition = targetPosition
        rightLift.targetPosition = targetPosition
        telemetry()
    }

    fun telemetry() {
        telemetry.addData("Left Stick Y", Mercurial.gamepad2.leftStickY.state)
        telemetry.addData("Left Lift Real Position", leftLift.currentPosition)
        telemetry.addData("Right Lift Real Position", rightLift.currentPosition)
        telemetry.addData("$subsystemName Position", targetPosition)
        telemetry.addData("Left Lift Power (real vs intended)", "${leftLift.power}")
        telemetry.addData("Right Lift Power (real vs intended)", "${rightLift.power}")
    }
}