package org.firstinspires.ftc.teamcode.util.structures

class Timer {
    private val stopwatch = Stopwatch()
    private var time = 0L

    fun start(wait: Long) {
        stopwatch.start()
        time = wait
    }

    private fun elapsedTime(): Long {
        return stopwatch.elapsedTime()
    }

    fun isFinished(): Boolean {
        return elapsedTime() >= time
    }
}