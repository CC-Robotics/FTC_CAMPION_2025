package org.firstinspires.ftc.teamcode.subsystem.outdated

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.controller.PIDFController
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@Suppress("unused")
@Config
object FieldCentricDrivetrainSubsystem : SubsystemCore() {
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

    private val imu by subsystemCell { getHardware<IMU>("imu") }

    @JvmField val xControl = PIDFController()
    @JvmField val yControl = PIDFController()
    @JvmField val thetaControl = PIDFController()

    @JvmField val botPose = Pose2D(DistanceUnit.CM, 0.0, 0.0, AngleUnit.DEGREES, 0.0)
    @JvmField val targetPose = Pose2D(DistanceUnit.CM, 0.0, 0.0, AngleUnit.DEGREES, 0.0)

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
        defaultCommand = update()
    }

    fun applyPower(x: Double, y: Double, rx: Double) {
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

    private fun drive(x: Double, y: Double, rx: Double) {
        applyPower(x * 1.1, -y, rx)
    }

    fun update(): Lambda {
        return Lambda("Field Centric Drive")
            .addRequirements(this::class.java)
            .setExecute {
                val xPower = xControl.calculate(botPose.getX(DistanceUnit.CM), targetPose.getX(DistanceUnit.CM))
                val yPower = yControl.calculate(botPose.getX(DistanceUnit.CM), targetPose.getY(DistanceUnit.CM))
                val thetaPower = thetaControl.calculate(AngleUnit.normalizeDegrees(botPose.getHeading(AngleUnit.DEGREES)), AngleUnit.normalizeDegrees(targetPose.getHeading(AngleUnit.RADIANS)))
            }
            .addInterruptible { true }
            .setFinish { false }
    }
}