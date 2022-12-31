package frc.robot.sim

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import frc.robot.sim.PhysicsSim.Companion.random
import frc.robot.sim.PhysicsSim.SimProfile
import kotlin.math.abs

// random()
/**
 * Holds information about a simulated TalonSRX.
 *
 * @constructor Creates a new simulation profile for a TalonSRX device.
 * @param talon
 * The TalonSRX device
 * @param accelToFullTime
 * The time the motor takes to accelerate from 0 to full, in seconds
 * @param fullVel
 * The maximum motor velocity, in ticks per 100ms
 * @param sensorPhase
 * The phase of the TalonSRX sensors
 */
internal class TalonSRXSimProfile
(
    private val talon: TalonSRX,
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
    override fun run() {
        val period = period
        val accelAmount = fullVel / accelToFullTime * period / 1000

        /// DEVICE SPEED SIMULATION
        var outPerc = talon.simCollection.motorOutputLeadVoltage / 12
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
        talon.simCollection.addQuadraturePosition((_vel * period / 100).toInt())
        talon.simCollection.setQuadratureVelocity(_vel.toInt())
        val supplyCurrent: Double = abs(outPerc) * 30 * random(0.95, 1.05)
        val statorCurrent: Double = if (outPerc == 0.0) 0.0 else supplyCurrent / abs(outPerc)
        talon.simCollection.setSupplyCurrent(supplyCurrent)
        talon.simCollection.setStatorCurrent(statorCurrent)
        talon.simCollection.setBusVoltage(12 - outPerc * outPerc * 3 / 4 * random(0.95, 1.05))
    }
}