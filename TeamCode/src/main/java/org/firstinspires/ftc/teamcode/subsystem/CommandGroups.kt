package org.firstinspires.ftc.teamcode.subsystem

import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem.AXLE_INIT

object CommandGroups {
    fun retract(): Parallel {
        return Parallel(
            LiftSubsystem.retract(),
            GripperSubsystem.goTo(GripperSubsystem.ServoType.AXLE, AXLE_INIT)
        )
    }

    private fun goToCollectPosition(): Parallel {
        return Parallel(
            ArmSubsystem.goTo(302),
            LiftSubsystem.goTo(752).setFinish { true },
            GripperSubsystem.setClawState(GripperSubsystem.ClawState.CLOSED),
            GripperSubsystem.goTo(GripperSubsystem.ServoType.WRIST, 0.354),
            GripperSubsystem.goTo(GripperSubsystem.ServoType.AXLE, 0.304)
        )
    }

    fun collect(): Sequential {
        return Sequential(
            goToCollectPosition().raceWith(Wait(0.3)),
            DrivetrainSubsystem.lockIn()
        )
    }
}