package frc.robot.sim

import com.ctre.phoenix.sensors.CANCoder
import frc.robot.sim.PhysicsSim.SimProfile

internal class CANCoderSimProfile
(
    private val canCoder: CANCoder,
    private val sensorPhase: Boolean = false
) : SimProfile() {
    /** The current position  */
    private var _pos: Double = 0.0
    /** The current velocity  */
    private var _vel = 0.0

    /**
     * Runs the simulation profile.
     *
     * This uses very rudimentary physics simulation and exists to allow users to test
     * features of our products in simulation using our examples out of the box.
     * Users may modify this to utilize more accurate physics simulation.
     */
    override fun run(dt: Double?) {
        val period = if (dt == null) period else dt

        // calculate theoretical position
        val theoreticalPos = _pos + _vel * period / 1000

        // set simulated position
        canCoder.simCollection.setRawPosition(theoreticalPos.toInt())

        // set simulated velocity
        canCoder.simCollection.setVelocity(_vel.toInt())
    }

    /**
     * simulated position.
     */
    var pos: Double
        // convert
        get() = _pos / 4096.0
        set(value) {
            _pos = value * 4096.0
            canCoder.simCollection.setRawPosition((value * 4096.0).toInt())
        }

    /**
     * simulated velocity.
     */
    var vel: Double
        // convert
        get() = _vel / 4096.0
        set(value) {
            _vel = value * 4096.0
            canCoder.simCollection.setVelocity((value * 4096.0).toInt())
        }
}