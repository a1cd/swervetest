package frc.robot.sim

import com.ctre.phoenix.motorcontrol.can.VictorSPX
import frc.robot.sim.PhysicsSim.Companion.random
import frc.robot.sim.PhysicsSim.SimProfile

/**
 * Holds information about a simulated VictorSPX.
 * @param victor
 * The VictorSPX device
 */
internal class VictorSPXSimProfile
(val victor: VictorSPX) : SimProfile() {
    /**
     * Runs the simulation profile.
     *
     * This uses very rudimentary physics simulation and exists to allow users to test
     * features of our products in simulation using our examples out of the box.
     * Users may modify this to utilize more accurate physics simulation.
     */
    override fun run() {
        // final double period = getPeriod();

        // Device voltage simulation
        val outPerc = victor.simCollection.motorOutputLeadVoltage / 12
        victor.simCollection.setBusVoltage(12 - outPerc * outPerc * 3 / 4 * random(0.95, 1.05))
    }
}