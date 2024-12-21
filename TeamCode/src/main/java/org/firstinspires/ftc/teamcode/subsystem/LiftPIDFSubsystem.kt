package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import java.lang.annotation.Inherited

object LiftPIDFSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Lift"

    private val rightLift by getHardware<DcMotorEx>("right_lift")
    private val leftLift by getHardware<DcMotorEx>("left_lift")

    override fun init(opMode: Wrapper) {
        position = 0
        pidfController.setPIDF(0.003, 0.02, 0.0, 0.025)

        leftLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftLift.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightLift.mode = DcMotor.RunMode.RUN_USING_ENCODER

        leftLift.direction = DcMotorSimple.Direction.REVERSE
        rightLift.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun periodic(opMode: Wrapper) {
        changePosition(Mercurial.gamepad2.leftStickY.state).execute()
        val power = pidfController.calculate(rightLift.currentPosition.toDouble(), position.toDouble())

        leftLift.power = power
        rightLift.power = power

        telemetry.addData("Left Stick Y", Mercurial.gamepad2.leftStickY.state)
        telemetry.addData("Left Lift Real Position", leftLift.currentPosition)
        telemetry.addData("Right Lift Real Position", rightLift.currentPosition)
        telemetry.addData("$subsystemName Position", position)
        telemetry.addData("Left Lift Power (real vs intended)", "${leftLift.power} vs $power")
        telemetry.addData("Right Lift Power (real vs intended)", "${rightLift.power} vs $power")
    }
}