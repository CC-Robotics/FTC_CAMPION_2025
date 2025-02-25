package org.firstinspires.ftc.teamcode.utils

import dev.frozenmilk.util.units.distance.Distance
import dev.frozenmilk.util.units.distance.cm
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LinearSlideSubsystem
import kotlin.math.cos
import kotlin.math.sqrt

fun calculateEndEffectorDistance(lift: LiftSubsystem, slide: LinearSlideSubsystem): Distance {
    val angleRadians = lift.angle.intoRadians().value
    val extension = slide.length.value
    return sqrt(
        extension * extension + extension * extension - 2 * extension * extension * cos(
            angleRadians
        )
    ).cm
}