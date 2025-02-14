package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage
import kotlin.math.abs
import kotlin.math.sign

@Config
class TwoDeadWheelLocalizer(hardwareMap: HardwareMap, private val imu: IMU,
                            private val inPerTick: Double, pose: Pose2d) :
    Localizer {
    class Params {
        var parYTicks: Double = 0.0 // y position of the parallel encoder (in tick units)
        var perpXTicks: Double = 0.0 // x position of the perpendicular encoder (in tick units)
    }

    // TODO: make sure your config has **motors** with these names (or change them)
    //   the encoders should be plugged into the slot matching the named motor
    //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
    val par: Encoder = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "par")))
    val perp: Encoder = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "perp")))

    // TODO: reverse encoder directions if needed
    //   par.setDirection(DcMotorSimple.Direction.REVERSE);

    private var lastParPos = 0
    private var lastPerpPos = 0
    private var lastHeading: Rotation2d? = null

    private var lastRawHeadingVel = 0.0
    private var headingVelOffset = 0.0
    private var initialized = false
    override var pose: Pose2d?

    init {

        write("TWO_DEAD_WHEEL_PARAMS", PARAMS)

        this.pose = pose
    }

    override fun update(): PoseVelocity2d {
        val parPosVel: PositionVelocityPair = par.getPositionAndVelocity()
        val perpPosVel: PositionVelocityPair = perp.getPositionAndVelocity()

        val angles: YawPitchRollAngles = imu.robotYawPitchRollAngles
        // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
        val angularVelocityDegrees: AngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
        val angularVelocity: AngularVelocity = AngularVelocity(
            UnnormalizedAngleUnit.RADIANS,
            Math.toRadians(angularVelocityDegrees.xRotationRate.toDouble()).toFloat(),
            Math.toRadians(angularVelocityDegrees.yRotationRate.toDouble()).toFloat(),
            Math.toRadians(angularVelocityDegrees.zRotationRate.toDouble()).toFloat(),
            angularVelocityDegrees.acquisitionTime
        )

        write(
            "TWO_DEAD_WHEEL_INPUTS",
            TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity)
        )

        val heading: Rotation2d = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        val rawHeadingVel: Double = angularVelocity.zRotationRate.toDouble()
        if (abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= sign(rawHeadingVel) * 2 * Math.PI
        }
        lastRawHeadingVel = rawHeadingVel
        val headingVel = headingVelOffset + rawHeadingVel

        if (!initialized) {
            initialized = true

            lastParPos = parPosVel.position
            lastPerpPos = perpPosVel.position
            lastHeading = heading

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        val parPosDelta: Int = parPosVel.position - lastParPos
        val perpPosDelta: Int = perpPosVel.position - lastPerpPos
        val headingDelta: Double = heading.minus(lastHeading!!)

        val twist: Twist2dDual<Time> = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    doubleArrayOf(
                        parPosDelta - PARAMS.parYTicks * headingDelta,
                        parPosVel.velocity - PARAMS.parYTicks * headingVel,
                    )
                ).times(inPerTick),
                DualNum<Time>(
                    doubleArrayOf(
                        perpPosDelta - PARAMS.perpXTicks * headingDelta,
                        perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                    )
                ).times(inPerTick)
            ),
            DualNum(
                doubleArrayOf(
                    headingDelta,
                    headingVel,
                )
            )
        )

        lastParPos = parPosVel.position
        lastPerpPos = perpPosVel.position
        lastHeading = heading

        pose = pose!!.plus(twist.value())
        return twist.velocity().value()
    }

    companion object {
        var PARAMS: Params = Params()
    }
}
