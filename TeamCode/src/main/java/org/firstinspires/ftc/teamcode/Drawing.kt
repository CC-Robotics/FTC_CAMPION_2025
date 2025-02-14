package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.Pose2d

object Drawing {
    @JvmStatic
    fun drawRobot(c: Canvas, t: Pose2d) {
        val robotRadius = 9.0

        c.setStrokeWidth(1)
        c.strokeCircle(t.position.x, t.position.y, robotRadius)

        val halfv = t.heading.vec().times(0.5 * robotRadius)
        val p1 = t.position.plus(halfv)
        val p2 = p1.plus(halfv)
        c.strokeLine(p1.x, p1.y, p2.x, p2.y)
    }
}
