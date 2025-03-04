package org.firstinspires.ftc.teamcode.util.structures

class Stopwatch {
    private var startTime: Long = 0
    private var endTime: Long = 0
    private var running = false

    fun start() {
        startTime = System.currentTimeMillis()
        running = true
    }

    fun stop() {
        endTime = System.currentTimeMillis()
        running = false
    }

    fun elapsedTime(): Long {
        return if (running) {
            System.currentTimeMillis() - startTime
        } else {
            endTime - startTime
        }
    }
}