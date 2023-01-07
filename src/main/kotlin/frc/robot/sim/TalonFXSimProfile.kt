package frc.robot.sim

import com.ctre.phoenix.motorcontrol.can.TalonFX
import frc.robot.sim.PhysicsSim.Companion.random
import frc.robot.sim.PhysicsSim.SimProfile
import kotlin.math.abs

/**
 * Holds information about a simulated TalonFX.
 * @param falcon
 * The TalonFX device
 * @param accelToFullTime
 * The time the motor takes to accelerate from 0 to full, in seconds
 * @param fullVel
 * The maximum motor velocity, in ticks per 100ms
 * @param sensorPhase
 * The phase of the TalonFX sensors
 */
internal class TalonFXSimProfile
(
    private val falcon: TalonFX,
    private val accelToFullTime: Double,
    private val fullVel: Double,
    private val sensorPhase: Boolean
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
        val accelAmount = fullVel / accelToFullTime * period / 1000

        /// DEVICE SPEED SIMULATION
        var outPerc = falcon.simCollection.motorOutputLeadVoltage / 12
        if (sensorPhase) {
            outPerc *= -1.0
        }
        // Calculate theoretical velocity with some randomness
        val theoreticalVel: Double = outPerc * fullVel * random(0.95, 1.0)
        // Simulate motor load
        if (theoreticalVel > _vel + accelAmount) {
            _vel += accelAmount
        } else if (theoreticalVel < _vel - accelAmount) {
            _vel -= accelAmount
        } else {
            _vel += 0.9 * (theoreticalVel - _vel)
        }
        // _pos += _vel * period / 100;

        /// SET SIM PHYSICS INPUTS
        falcon.simCollection.addIntegratedSensorPosition((_vel * period / 100).toInt())
        falcon.simCollection.setIntegratedSensorVelocity(_vel.toInt())
        val supplyCurrent: Double = abs(outPerc) * 30 * random(0.95, 1.05)
        val statorCurrent: Double = if (outPerc == 0.0) 0.0 else supplyCurrent / abs(outPerc)
        falcon.simCollection.setSupplyCurrent(supplyCurrent)
        falcon.simCollection.setStatorCurrent(statorCurrent)
        falcon.simCollection.setBusVoltage(12 - outPerc * outPerc * 3 / 4 * random(0.95, 1.05))
    }
}