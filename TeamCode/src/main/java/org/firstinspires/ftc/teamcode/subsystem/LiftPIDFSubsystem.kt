package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
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

    override fun periodic(opMode: Wrapper) {
        applyPIDF(leftLift)
        applyPIDF(rightLift)
    }

    override fun init(opMode: Wrapper) {
        position = rightLift.currentPosition
    }

    init {
        pidfController.p = 0.02
    }
}