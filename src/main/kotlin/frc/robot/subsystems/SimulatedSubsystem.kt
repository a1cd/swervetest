package frc.robot.subsystems

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj2.command.SubsystemBase

open class SimulatedSubsystem(
    open var children: List<SimulatedSubsystem> = listOf()
): SubsystemBase() {
    val isSim: Boolean get() = !RobotBase.isReal()
    open val currentDraw: Double get() = children.map { it.currentDraw }.sum()

    open var lastTime = Timer.getFPGATimestamp()
    open fun simUpdate(dt: Double) {
        children.forEach {
            it.simUpdate(dt)
        }
    }
    fun simUpdate() {
        // get delta time
        val now = Timer.getFPGATimestamp()
        val dt = now - lastTime
        lastTime = now
        simUpdate(dt)
    }
    // static simUpdate method for top level simulation: includes battery
    companion object {
        fun simUpdate(vararg subsystems: SimulatedSubsystem, dt: Double? = null) {
            // update battery
            RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    subsystems.map { it.currentDraw }.sum()
                )
            )
            // update all subsystems
            subsystems.forEach { if (dt == null) it.simUpdate() else it.simUpdate(dt) }
        }
    }
}