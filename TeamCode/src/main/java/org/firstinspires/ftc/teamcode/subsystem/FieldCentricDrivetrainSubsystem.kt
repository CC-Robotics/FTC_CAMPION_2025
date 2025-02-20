package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

// This aint ur unc's AI technology
@Suppress("unused")
object FieldCentricDrivetrainSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val fR by getHardware<DcMotorEx>("fR")
    private val fL by getHardware<DcMotorEx>("fL")
    private val bR by getHardware<DcMotorEx>("bR")
    private val bL by getHardware<DcMotorEx>("bL")

    private val imu by getHardware<IMU>("imu")

    override fun init(opMode: Wrapper) {
        fR.direction = DcMotorSimple.Direction.REVERSE
        bR.direction = DcMotorSimple.Direction.REVERSE

        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        )

        imu.initialize(parameters)
    }

    override fun periodic(opMode: Wrapper) {
        val gamepad1 = Mercurial.gamepad1
        val y = -gamepad1.leftStickY.state // Remember, this is reversed!
        val x = gamepad1.leftStickX.state * 1.1 // Counteract imperfect strafing
        val rx = gamepad1.rightStickX.state

        if (gamepad1.options.state || gamepad1.start.state) imu.resetYaw()

        val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

        // Rotate movement direction counter to the bot's rotation
        var rotX = x * cos(-heading) - y * sin(-heading)
        val rotY = x * sin(-heading) + y * cos(-heading)

        rotX *= 1.1

        val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
        val fLPower = (rotY + rotX - rx) / denominator
        val bLPower = (rotY - rotX - rx) / denominator
        val fRPower = (rotY - rotX + rx) / denominator
        val bRPower = (rotY + rotX + rx) / denominator

        fL.power = fLPower
        bL.power = bLPower
        fR.power = fRPower
        bR.power = bRPower
    }
}