package frc.robot.subsystems

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.sim.PhysicsSim

open class SimulatedSubsystem(
    open var children: List<SimulatedSubsystem> = listOf()
): SubsystemBase() {
    constructor(vararg children: SimulatedSubsystem): this(children.toList())
    val isSim: Boolean = RobotBase.isSimulation()
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
            PhysicsSim.instance.run()
        }
    }
}