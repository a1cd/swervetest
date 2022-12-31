package frc.robot.sim

import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import java.lang.Math.random
import kotlin.math.IEEErem
import kotlin.math.sin

/**
 * Manages physics simulation for CTRE products.
 */
class PhysicsSim {
    /**
     * Adds a TalonSRX controller to the simulator.
     *
     * @param talon
     * The TalonSRX device
     * @param accelToFullTime
     * The time the motor takes to accelerate from 0 to full, in seconds
     * @param fullVel
     * The maximum motor velocity, in ticks per 100ms
     * @param sensorPhase
     * The phase of the TalonSRX sensors
     */
    @JvmOverloads
    fun addTalonSRX(talon: TalonSRX?, accelToFullTime: Double, fullVel: Double, sensorPhase: Boolean = false) {
        if (talon != null) {
            val simTalon = TalonSRXSimProfile(talon, accelToFullTime, fullVel, sensorPhase)
            simProfiles.add(simTalon)
        }
    }
    /**
     * Adds a TalonFX controller to the simulator.
     *
     * @param falcon
     * The TalonFX device
     * @param accelToFullTime
     * The time the motor takes to accelerate from 0 to full, in seconds
     * @param fullVel
     * The maximum motor velocity, in ticks per 100ms
     * @param sensorPhase
     * The phase of the TalonFX sensors
     */
    @JvmOverloads
    fun addTalonFX(falcon: TalonFX?, accelToFullTime: Double, fullVel: Double, sensorPhase: Boolean = false) {
        if (falcon != null) {
            val simFalcon = TalonFXSimProfile(falcon, accelToFullTime, fullVel, sensorPhase)
            simProfiles.add(simFalcon)
        }
    }

    /**
     * Adds a VictorSPX controller to the simulator.
     *
     * @param victor
     * The VictorSPX device
     */
    fun addVictorSPX(victor: VictorSPX?) {
        if (victor != null) {
            val simVictor = VictorSPXSimProfile(victor)
            simProfiles.add(simVictor)
        }
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate sensors
     */
    fun run() {
        // Simulate devices
        for (simProfile in simProfiles) {
            simProfile.run()
        }
    }

    private val simProfiles = ArrayList<SimProfile>()

    fun reset() {
        simProfiles.clear()
    }

    /**
     * Holds information about a simulated device.
     */
    internal open class SimProfile {
        private var _lastTime: Long = 0
        private var _running = false

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        open fun run() {}
        /**
         * The time since last call, in milliseconds.
         */
        protected val period: Double
            get() {
                // set the start time if not yet running
                if (!_running) {
                    _lastTime = System.nanoTime()
                    _running = true
                }
                val now = System.nanoTime()
                val period = (now - _lastTime) / 1000000.0
                _lastTime = now
                return period
            }
    }

    companion object {
        /**
         * Gets the robot simulator instance.
         */
        val instance = PhysicsSim()

        /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
        fun random(min: Double, max: Double): Double {
            return (max - min) / 2 * sin(random().IEEErem(2 * 3.14159)) + (max + min) / 2
        }

        /**
         * random(max) is equivalent to random(0, max)
         */
        fun random(max: Double): Double {
            return random(0.0, max)
        }
    }
}