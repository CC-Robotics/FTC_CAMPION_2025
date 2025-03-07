package org.firstinspires.ftc.teamcode.controller

import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.round
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min

/*
 * How to Tune PIDF Values:
 *   Simple guide on how to tune PIDF values in our various subsystems.
 *
 * 1. Understand PIDF:
 *    - P (Proportional):
 *      Controls response to error. Start low and increase until
 *      oscillation starts, then back off slightly.
 *
 *    - I (Integral):
 *      Corrects accumulated error over time. Increase it if the
 *      system struggles to reach the target, but don't overdo it,
 *      as it can cause instability.
 *
 *    - D (Derivative):
 *      Dampens changes. Helps smooth the response. Start with zero
 *      and slowly increase until the system becomes stable.
 *
 *    - F (Feedforward):
 *      Directly scales the target value. Use it to offset known
 *      constant forces (e.g., gravity, friction). This is usually
 *      calculated based on system modeling.
 *
 * 2. Steps to Tune:
 *    a. Start with only P (set I, D, F to 0):
 *       - Gradually increase P until the system starts oscillating around the target.
 *       - Once it oscillates, reduce P slightly until it stabilizes.
 *    b. Add I:
 *       - Slowly increase I to help the system reach the target accurately.
 *       - Stop when you observe minimal overshooting or oscillation.
 *    c. Add D:
 *       - Increase D to reduce overshoot and stabilize response further.
 *       - Too much D can make the system sluggish, so balance is key.
 *    d. Adjust F (if applicable):
 *       - Calculate or estimate the F value based on known forces acting on the system.
 *       - Use this as a starting point and fine-tune for better control.
 *
 * 3. Test and Iterate:
 *    - Test the system with realistic targets and scenarios.
 *    - Fine-tune each value as needed for smooth, accurate, and stable performance.
 *
 * 4. Common Pitfalls:
 *    - Over-tuning (too high values): Causes instability or oscillation.
 *    - Under-tuning (too low values): Results in sluggish or inaccurate response.
 *    - Ignoring system-specific dynamics: Always account for the physical characteristics of your system.
 *
 * PID tuning is as much an art as it is a science.
 */


/**
 * This is a PID controller (https://en.wikipedia.org/wiki/PID_controller)
 * for your robot. Internally, it performs all the calculations for you.
 * You need to tune your values to the appropriate amounts in order
 * to properly utilize these calculations.
 *
 *
 * The equation we will use is:
 * u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF
 * where e(t) = r(t) - y(t) and r(t) is the setpoint and y(t) is the
 * measured value. If we consider e(t) the positional error, then
 * int(0,t)[e(t')dt'] is the total error and e'(t) is the velocity error.
 */
class PIDFController @JvmOverloads constructor(
    var p: Double = 0.0,
    var i: Double = 0.0,
    var d: Double = 0.0,
    var f: Double = 0.0,
    private var setPoint: Double = 0.0,
    private var measuredValue: Double = 0.0,
    private var useCosineFeedforward: Boolean = false,
    ffOffset: Double = 0.0
) {
    private var minIntegral: Double
    private var maxIntegral = 1.0
    private var ffOffset = ffOffset * (Math.PI / 180.0)

    private val ticksInDegree = 700 / 180.0;

    fun setFFOffset(ffOffset: Double) {
        this.ffOffset = ffOffset * (Math.PI / 180.0)
    }

    /**
     * @return the positional error e(t)
     */
    var positionError: Double
        private set

    /**
     * @return the velocity error e'(t)
     */
    var velocityError: Double = 0.0
        private set

    private var totalError = 0.0
    private var prevErrorVal = 0.0

    private var errorTolerance_p = 0.05
    private var errorTolerance_v = Double.POSITIVE_INFINITY

    private var lastTimeStamp = 0.0
    var period: Double = 0.0
        private set

    /**
     * This is the full constructor for the PIDF controller. Our PIDF controller
     * includes a feed-forward value which is useful for fighting friction and gravity.
     * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
     *
     * @param setPoint The setpoint of the pid control loop.
     * @param measuredValue The measured value of he pid control loop. We want sp = pv, or to the degree
     * such that sp - pv, or e(t) < tolerance.
     */
    /**
     * The base constructor for the PIDF controller
     */
    init {
        minIntegral = -1.0

        positionError = setPoint - measuredValue
        reset()
    }

    fun reset() {
        totalError = 0.0
        prevErrorVal = 0.0
        lastTimeStamp = 0.0
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
        errorTolerance_p = positionTolerance
        errorTolerance_v = velocityTolerance
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    fun getSetPoint(): Double {
        return setPoint
    }

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     */
    fun setSetPoint(sp: Double) {
        setPoint = sp
        positionError = setPoint - measuredValue
        velocityError = (positionError - prevErrorVal) / period
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * [.setTolerance].
     *
     * @return Whether the error is within the acceptable bounds.
     */
    fun atSetPoint(): Boolean {
        return abs(positionError) < errorTolerance_p
                && abs(velocityError) < errorTolerance_v
    }

    val coefficients: DoubleArray
        /**
         * @return the PIDF coefficients
         */
        get() = doubleArrayOf(p, i, d, f)

    val tolerance: DoubleArray
        /**
         * @return the tolerances of the controller
         */
        get() = doubleArrayOf(errorTolerance_p, errorTolerance_v)

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @param pv The given measured value.
     * @param sp The given setpoint.
     * @return the next output using the given measurd value via
     * [.calculate].
     */
    fun calculate(pv: Double, sp: Double): Double {
        // set the setpoint to the provided value
        setSetPoint(sp)
        return calculate(pv)
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param measuredValue The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via
     * [.calculate].
     */
    @JvmOverloads
    fun calculate(pv: Double = measuredValue): Double {
        prevErrorVal = positionError

        val currentTimeStamp = System.nanoTime().toDouble() / 1E9
        if (lastTimeStamp == 0.0) lastTimeStamp = currentTimeStamp
        period = currentTimeStamp - lastTimeStamp
        lastTimeStamp = currentTimeStamp

        if (measuredValue == pv) {
            positionError = setPoint - measuredValue
        } else {
            positionError = setPoint - pv
            measuredValue = pv
        }

        if (abs(period) > 1E-6) {
            velocityError = (positionError - prevErrorVal) / period
        } else {
            velocityError = 0.0
        }

        /*
        if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        totalError += period * (setPoint - measuredValue)
        totalError = if (totalError < minIntegral) minIntegral else min(maxIntegral, totalError)

        // returns u(t)
        if (useCosineFeedforward) {
            val fm = cos(Math.toRadians(setPoint / ticksInDegree) + ffOffset)
            return p * positionError + i * totalError + d * velocityError + f * fm * setPoint
        }
        val ff = cos(Math.toRadians(setPoint / ticksInDegree))
        return p * positionError + i * totalError + d * velocityError + f * setPoint
    }

    fun setPIDF(kp: Double, ki: Double, kd: Double, kf: Double) {
        p = kp
        i = ki
        d = kd
        f = kf
    }

    fun setPIDF(k: PIDFCoefficients) {
        p = k.p
        i = k.i
        d = k.d
        f = k.f
    }

    fun setIntegrationBounds(integralMin: Double, integralMax: Double) {
        minIntegral = integralMin
        maxIntegral = integralMax
    }

    fun clearTotalError() {
        totalError = 0.0
    }
}