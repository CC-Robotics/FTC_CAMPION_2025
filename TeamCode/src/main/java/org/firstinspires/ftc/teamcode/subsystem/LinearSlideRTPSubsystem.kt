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

object LinearSlideRTPSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Slide"

    private val slide by getHardware<DcMotorEx>("slide")

    private const val MAX_POSITION = 600

    override fun init(opMode: Wrapper) {
        position = 0
        slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide.targetPosition = 0
        slide.mode = DcMotor.RunMode.RUN_TO_POSITION
        slide.direction = DcMotorSimple.Direction.REVERSE
    }

    fun update(increment: Double) {
        changePosition(increment)
        clampPosition(0, MAX_POSITION)
        slide.targetPosition = position
        telemetry()
    }

    fun telemetry() {
        telemetry.addData("$subsystemName Real Position", slide.currentPosition)
        telemetry.addData("$subsystemName Position", position)
        telemetry.addData("$subsystemName Power", "${slide.power}")
    }

    enum class SlidePosition(val position: Int) {
        VISION_POSITION(300)
    }
}