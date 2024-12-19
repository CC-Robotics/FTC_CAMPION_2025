package org.firstinspires.ftc.teamcode.structures

import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem.subsystemCell

open class SubsystemCore : Subsystem {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY

    open val subsystemName = this::class.simpleName

    companion object {
        inline fun <reified T, reified K> getHardwareAndCast(id: String): SubsystemObjectCell<K> = subsystemCell {
            val hardware = FeatureRegistrar.activeOpMode.hardwareMap.get(T::class.java, id)
            return@subsystemCell hardware as K
        }

        inline fun <reified T> getHardware(id: String): SubsystemObjectCell<T> = subsystemCell {
            FeatureRegistrar.activeOpMode.hardwareMap.get(T::class.java, id)
        }
    }

    override fun preUserInitHook(opMode: Wrapper) {
        init(opMode)
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        periodic(opMode)
    }

    open fun init(opMode: Wrapper) {}

    open fun periodic(opMode: Wrapper) {}
}