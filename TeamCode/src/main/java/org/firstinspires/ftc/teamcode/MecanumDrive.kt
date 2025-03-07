package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryActionFactory
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnActionFactory
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.messages.PoseMessage
import java.util.Arrays
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.max

@Config
class MecanumDrive(hardwareMap: HardwareMap, pose: Pose2d?) {
    class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP
        var usbFacingDirection: RevHubOrientationOnRobot.UsbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT

        // drive model parameters
        var inPerTick: Double = 1.0
        var lateralInPerTick: Double = inPerTick
        var trackWidthTicks: Double = 0.0

        // feedforward parameters (in tick units)
        var kS: Double = 0.0
        var kV: Double = 0.0
        var kA: Double = 0.0

        // path profile parameters (in inches)
        var maxWheelVel: Double = 50.0
        var minProfileAccel: Double = -30.0
        var maxProfileAccel: Double = 50.0

        // turn profile parameters (in radians)
        var maxAngVel: Double = Math.PI // shared with path
        var maxAngAccel: Double = Math.PI

        // path controller gains
        var axialGain: Double = 0.0
        var lateralGain: Double = 0.0
        var headingGain: Double = 0.0 // shared with turn

        var axialVelGain: Double = 0.0
        var lateralVelGain: Double = 0.0
        var headingVelGain: Double = 0.0 // shared with turn
    }

    val kinematics: MecanumKinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick
    )

    val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    )
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        Arrays.asList<VelConstraint>(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    val voltageSensor: VoltageSensor

    val lazyImu: LazyImu

    val localizer: Localizer
    private val poseHistory: LinkedList<Pose2d?> = LinkedList<Pose2d?>()

    private val estimatedPoseWriter: DownsampledWriter =
        DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter: DownsampledWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter: DownsampledWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter: DownsampledWriter =
        DownsampledWriter("MECANUM_COMMAND", 50000000)

    inner class DriveLocalizer(pose: Pose2d) : Localizer {
        val leftFront: Encoder
        val leftBack: Encoder
        val rightBack: Encoder
        val rightFront: Encoder
        val imu: IMU

        private var lastLeftFrontPos = 0
        private var lastLeftBackPos = 0
        private var lastRightBackPos = 0
        private var lastRightFrontPos = 0
        private var lastHeading: Rotation2d? = null
        private var initialized = false
        override var pose: Pose2d?

        init {
            leftFront = OverflowEncoder(RawEncoder(this@MecanumDrive.leftFront))
            leftBack = OverflowEncoder(RawEncoder(this@MecanumDrive.leftBack))
            rightBack = OverflowEncoder(RawEncoder(this@MecanumDrive.rightBack))
            rightFront = OverflowEncoder(RawEncoder(this@MecanumDrive.rightFront))

            imu = lazyImu.get()

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            this.pose = pose
        }

        override fun update(): PoseVelocity2d {
            val leftFrontPosVel: PositionVelocityPair = leftFront.getPositionAndVelocity()
            val leftBackPosVel: PositionVelocityPair = leftBack.getPositionAndVelocity()
            val rightBackPosVel: PositionVelocityPair = rightBack.getPositionAndVelocity()
            val rightFrontPosVel: PositionVelocityPair = rightFront.getPositionAndVelocity()

            val angles: YawPitchRollAngles = imu.getRobotYawPitchRollAngles()

            write(
                "MECANUM_LOCALIZER_INPUTS", MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles
                )
            )

            val heading: Rotation2d = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

            if (!initialized) {
                initialized = true

                lastLeftFrontPos = leftFrontPosVel.position
                lastLeftBackPos = leftBackPosVel.position
                lastRightBackPos = rightBackPosVel.position
                lastRightFrontPos = rightFrontPosVel.position

                lastHeading = heading

                return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            }

            val headingDelta: Double = heading.minus(lastHeading!!)
            val twist: Twist2dDual<Time> = kinematics.forward<Time>(
                MecanumKinematics.WheelIncrements<Time>(
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftFrontPosVel.position - lastLeftFrontPos).toDouble(),
                            leftFrontPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftBackPosVel.position - lastLeftBackPos).toDouble(),
                            leftBackPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightBackPosVel.position - lastRightBackPos).toDouble(),
                            rightBackPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightFrontPosVel.position - lastRightFrontPos).toDouble(),
                            rightFrontPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick)
                )
            )

            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftBackPos = leftBackPosVel.position
            lastRightBackPos = rightBackPosVel.position
            lastRightFrontPos = rightFrontPosVel.position

            lastHeading = heading

            pose = pose!!.plus(
                Twist2d(
                    twist.line.value(),
                    headingDelta
                )
            )

            return twist.velocity().value()
        }
    }

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "fL")
        leftBack = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "bL")
        rightBack = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "bR")
        rightFront = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "fR")

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: reverse motor directions if needed
        leftFront.direction = DcMotorSimple.Direction.REVERSE;
        leftBack.direction = DcMotorSimple.Direction.REVERSE;

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = LazyImu(
            hardwareMap, "imu", RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection
            )
        )

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        localizer = DriveLocalizer(pose!!)

        write("MECANUM_PARAMS", PARAMS)
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels: MecanumKinematics.WheelVelocities<Time> = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual.constant(powers, 1)
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    inner class FollowTrajectoryAction(t: TimeTrajectory) : Action {
        val timeTrajectory: TimeTrajectory = t
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps: List<Double> = range(
                0.0, t.path.length(),
                max(2.0, (ceil(t.path.length() / 2) as Int).toDouble()).toInt()
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p: Pose2d = t.path.get(disps[i], 1).value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = com.acmerobotics.roadrunner.now()
                t = 0.0
            } else {
                t = com.acmerobotics.roadrunner.now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0.0)
                leftBack.setPower(0.0)
                rightBack.setPower(0.0)
                rightFront.setPower(0.0)

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = timeTrajectory.get(t)
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot: PoseVelocity2d? = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, localizer.pose!!, robotVelRobot!!)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse<Time>(command)
            val voltage: Double = voltageSensor.getVoltage()

            val feedforward: MotorFeedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower: Double = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower: Double = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower: Double = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower: Double = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            p.put("x", localizer.pose!!.position.x)
            p.put("y", localizer.pose!!.position.y)
            p.put("heading (deg)", Math.toDegrees(localizer.pose!!.heading.toDouble()))

            val error: Pose2d = txWorldTarget.value().minusExp(localizer.pose!!)
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c: Canvas = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.pose!!)

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#4CAF507A")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(turn: TimeTurn) : Action {
        private val turn: TimeTurn = turn

        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = com.acmerobotics.roadrunner.now()
                t = 0.0
            } else {
                t = com.acmerobotics.roadrunner.now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.setPower(0.0)
                leftBack.setPower(0.0)
                rightBack.setPower(0.0)
                rightFront.setPower(0.0)

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = turn.get(t)
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot: PoseVelocity2d? = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, localizer.pose!!, robotVelRobot!!)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse<Time>(command)
            val voltage: Double = voltageSensor.getVoltage()
            val feedforward: MotorFeedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower: Double = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower: Double = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower: Double = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower: Double = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            val c: Canvas = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.pose!!)

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#7C4DFF7A")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun updatePoseEstimate(): PoseVelocity2d? {
        val vel: PoseVelocity2d? = localizer.update()
        poseHistory.add(localizer.pose)

        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(localizer.pose!!))


        return vel
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t!!.position.x
            yPoints[i] = t.position.y

            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn: TimeTurn -> TurnAction(turn) },
            { t: TimeTrajectory -> FollowTrajectoryAction(t) },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25, 0.1, 1e-2
                )
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }

    companion object {
        var PARAMS: Params = Params()
    }
}
