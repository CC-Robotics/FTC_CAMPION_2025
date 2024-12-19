package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import java.lang.annotation.Inherited

object LinearSlidePIDFSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Linear Slide"

    private val slide by getHardware<DcMotorEx>("slide")

    override fun periodic(opMode: Wrapper) {
        applyPIDF(slide)
        telemetry.addData("Slide Real Position", slide.currentPosition)
        telemetry.addData("$subsystemName} Position", position)
    }

    override fun init(opMode: Wrapper) {
        slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide.direction = DcMotorSimple.Direction.REVERSE
    }
}