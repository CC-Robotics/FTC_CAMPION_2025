package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.max

object DrivetrainSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val fR by subsystemCell { getHardware<DcMotorEx>("fR") }
    private val fL by subsystemCell { getHardware<DcMotorEx>("fL") }
    private val bR by subsystemCell { getHardware<DcMotorEx>("bR") }
    private val bL by subsystemCell { getHardware<DcMotorEx>("bL") }

    override fun init(opMode: Wrapper) {
        fR.direction = DcMotorSimple.Direction.REVERSE
        bR.direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(x: Double = 0.0, y: Double = 0.0, rx: Double = 0.0) {
        // Normalize the values so neither exceed +/- 1.0
        val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
        val frontLeftPower = (y + x + rx) / denominator
        val backLeftPower = (y - x + rx) / denominator
        val frontRightPower = (y - x - rx) / denominator
        val backRightPower = (y + x - rx) / denominator

        // Send calculated power to wheels
        fL.power = frontLeftPower
        bL.power = backLeftPower
        fR.power = frontRightPower
        bR.power = backRightPower
    }

    fun updateMotors() {
        val gamepad1 = Mercurial.gamepad1
        val y = -gamepad1.leftStickY.state // Remember, this is reversed!
        val x = gamepad1.leftStickX.state * 1.1 // Counteract imperfect strafing
        val rx = gamepad1.rightStickX.state

        drive(x, y, rx)
    }
}