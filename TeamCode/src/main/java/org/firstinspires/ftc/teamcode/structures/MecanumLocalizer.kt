import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

data class Pose2D(var x: Double, var y: Double, var theta: Double)

class MecanumLocalizer(
    private val motorFL: DcMotorEx,  // Front Left
    private val motorFR: DcMotorEx,  // Front Right
    private val motorBL: DcMotorEx,  // Back Left
    private val motorBR: DcMotorEx,  // Back Right
    private val wheelRadius: Double, // Wheel radius in meters
    private val ticksPerRev: Double, // Encoder ticks per wheel revolution
    private val wheelBase: Double,   // Distance from center to front/back wheels (m)
    private val trackWidth: Double   // Distance from center to left/right wheels (m)
) {
    private var pose = Pose2D(0.0, 0.0, 0.0) // Robot pose (x, y, θ)
    private var lastTicks = intArrayOf(0, 0, 0, 0)
    private var lastTime = System.nanoTime()

    // ✅ Get wheel velocities (rad/s) using encoders
    private fun getWheelVelocities(): DoubleArray {
        val currentTicks = intArrayOf(
            motorFL.currentPosition,
            motorFR.currentPosition,
            motorBL.currentPosition,
            motorBR.currentPosition
        )

        val currentTime = System.nanoTime()
        val dt = (currentTime - lastTime) / 1e9 // Convert ns to seconds

        val wheelVelocities = DoubleArray(4)

        for (i in 0..3) {
            val deltaTicks = currentTicks[i] - lastTicks[i]
            val omega = (deltaTicks * 2 * PI) / (ticksPerRev * dt) // rad/s
            wheelVelocities[i] = omega
        }

        lastTicks = currentTicks
        lastTime = currentTime

        return wheelVelocities
    }

    // ✅ Convert wheel velocities into robot velocity (vx, vy, omegaZ)
    private fun getRobotVelocity(): Triple<Double, Double, Double> {
        val wheelVelocities = getWheelVelocities()
        val r = wheelRadius

        val vx = (r / 4) * (wheelVelocities[0] + wheelVelocities[1] + wheelVelocities[2] + wheelVelocities[3])
        val vy = (r / 4) * (-wheelVelocities[0] + wheelVelocities[1] + wheelVelocities[2] - wheelVelocities[3])
        val omegaZ = (r / (4 * (wheelBase + trackWidth))) * (-wheelVelocities[0] + wheelVelocities[1] - wheelVelocities[2] + wheelVelocities[3])

        return Triple(vx, vy, omegaZ)
    }

    // ✅ Updates robot pose (x, y, theta) over time
    fun updatePose() {
        val (vx, vy, omegaZ) = getRobotVelocity()
        val dt = (System.nanoTime() - lastTime) / 1e9 // Time delta (s)

        // Update pose using velocity and rotation
        pose.x += (vx * cos(pose.theta) - vy * sin(pose.theta)) * dt
        pose.y += (vx * sin(pose.theta) + vy * cos(pose.theta)) * dt
        pose.theta += omegaZ * dt
    }

    // ✅ Get current pose
    fun getPose(): Pose2D {
        return pose
    }
}
