package org.firstinspires.ftc.teamcode.subsystem

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.HardwareMap

// Leave the suppression until we integrate... Android studio will complain
@Suppress("unused")
class ArmSubsystem(hardwareMap: HardwareMap): SubsystemBase() {
    private val upperJoint: Motor = Motor(hardwareMap, "armUpper")
    private val lowerJoint: Motor = Motor(hardwareMap, "armLower")

    init {
        upperJoint.setRunMode(Motor.RunMode.PositionControl)
        lowerJoint.setRunMode(Motor.RunMode.PositionControl)
    }
//    override fun periodic() {
//        // This method will be called once per scheduler run
//    }
}