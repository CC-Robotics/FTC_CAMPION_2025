package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.controller.PIDFController
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

    override val pidfController = PIDFController(0.001, 0.0, 0.0, 0.0)


    private val rightLift by getHardware<DcMotorEx>("right_lift")
    private val leftLift by getHardware<DcMotorEx>("left_lift")

    override fun periodic(opMode: Wrapper) {
        applyPIDF(leftLift)
        applyPIDF(rightLift)
        telemetry.addData("Left Lift Real Position", leftLift.currentPosition)
        telemetry.addData("Right Lift Real Position", rightLift.currentPosition)
        telemetry.addData("$subsystemName Position", position)
    }

    override fun init(opMode: Wrapper) {
        leftLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftLift.direction = DcMotorSimple.Direction.REVERSE
        leftLift.direction = DcMotorSimple.Direction.REVERSE
    }
}