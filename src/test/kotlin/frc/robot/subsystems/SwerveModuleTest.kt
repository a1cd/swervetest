package frc.robot.subsystems


import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.sim.PhysicsSim
import frc.robot.subsystems.SimulatedSubsystem.Companion.simUpdate
import frc.robot.util.SimulatedTest
import org.hamcrest.CoreMatchers.notNullValue
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assume
import org.junit.Test
import kotlin.math.abs
import kotlin.system.exitProcess


/**
 * Test the swerve module.
 */
class SwerveModuleTest {

    val DELTA = 2e-2 // acceptable deviation range
    var swerveModule: SwerveModule = SwerveModule("test", 1, 2, 3, Translation2d(1.0, 1.0))


    @After
    fun tearDown() {
        PhysicsSim.instance.reset()
    }

    @Test
    fun getDriveMotor() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertNotNull(swerveModule.driveMotor)
    }

    @Test
    fun getSteerMotor() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertNotNull(swerveModule.steerMotor)
    }

    @Test
    fun getEnc() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertNotNull(swerveModule.enc)
    }

    @Test
    fun getAngle() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertEquals(0.0, swerveModule.angle, DELTA)
    }

    @Test
    fun getVelocity() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertEquals(0.0, swerveModule.velocity, DELTA)
    }

    @Test
    @SimulatedTest
    fun move() {
        Assume.assumeThat(swerveModule, notNullValue())
        swerveModule.move(SwerveModuleState(1.1, Rotation2d(1.2)))
        for (i in 0..100) {
            // run the command scheduler and physics simulation

            // run the move method on the swerve module, passing in a drive value of 1.0 and an angle value of 1.0
            swerveModule.move(SwerveModuleState(1.1, Rotation2d(1.2)))
        }
        println("swerveModule!!.angle: " + swerveModule.angle)
        println("swerveModule!!.velocity: " + swerveModule.velocity)
        println("swerveModule!!.translation2d: " + swerveModule.translation2d)
        println(swerveModule.driveMotor.motorOutputPercent)
        assertEquals(1.0, swerveModule.velocity, DELTA)
        assertEquals(1.0, swerveModule.angle, DELTA)
    }

    @Test
    fun testMove() {
        // run the simulation and the swerve module's move method 100 times
        for (i in 0..1000000) {
            // run the command scheduler and physics simulation
            simUpdate(swerveModule, dt = 0.02)
            // run the move method on the swerve module, passing in a drive value of 1.0 and an angle value of 1.0
            swerveModule.move(1.0, 1.0)
        }
        // assert that the velocity and angle of the swerve module are as expected
        assertEquals("swerve module speed should be 1.0", 1.0, swerveModule.velocity, DELTA)
        assertEquals(1.0, swerveModule.angle, DELTA)
    }


    @Test
    fun test() {
        (0..100).toList().map { it*0.00001+.1240}.forEach { P1: Double ->
            (0..100).toList().map{it*0.000001+.23}.forEach { P2: Double ->
            val velocityValues = mutableListOf<Double>()
                val swerveModule = SwerveModule("test", 1, 2, 3, Translation2d(1.0, 1.0))
                swerveModule.drivePid.p = P1.toDouble()
                swerveModule.drivePid.i = P2.toDouble()
                for (i in 0..10) {
                    swerveModule.move(1.0, 1.0)
                    simUpdate(swerveModule, dt = 0.02)
                    velocityValues += swerveModule.velocity
                }
                var hitOne = -1
                var failed = false
                var fail = ""
                velocityValues.forEachIndexed { index, value ->
                    if (index > 0) {
                        val previousValue = velocityValues[index - 1]
                        if (abs(value) >= 1.0 && hitOne== -1) {
                            hitOne = index
                        }
                        if (abs(value) >= 1.0) {
                            if (abs(value) < abs(previousValue) && !failed) { // if the velocity is decreasing
                                failed = true
                                fail = "velocity decreased at index $index"
                            }
                        }

                    }
                }
                if (!failed) {
                    println("P: $P1")
                    println("P: $P2")
                    println("velocityValues: ${velocityValues.map { it.toString() + ">\n" }.joinToString { it }}")
                    exitProcess(0)
                }
                if (failed) {
                    println("P: $P1")
                    println("P: $P2")
                    println(fail)
                    println("hitOne: $hitOne")
                }
            }
        }
    }
    @Test
    fun zeroEncoders() {
        swerveModule.zeroEncoders()
        assertEquals(0.0, swerveModule.angle, DELTA)
    }

    @Test
    fun reset() {

        swerveModule.reset()
        assertEquals(0.0, swerveModule.angle, DELTA)
        assertEquals(0.0, swerveModule.velocity, DELTA)
    }

    @Test
    fun periodic() {
        swerveModule.periodic()
        assertEquals(0.0, swerveModule.angle, DELTA)
        assertEquals(0.0, swerveModule.velocity, DELTA)
    }

    @Test
    fun simulationPeriodic() {
        swerveModule.simulationPeriodic()
        assertEquals(0.0, swerveModule.angle, DELTA)
        assertEquals(0.0, swerveModule.velocity, DELTA)
    }

    @Test
    @SimulatedTest
    fun stop() {
        swerveModule.stop()
        Thread.sleep(100)
        assertEquals(0.0, swerveModule.velocity, DELTA)
    }

    @Test
    fun getTranslation2d() {
        assertEquals(Translation2d(1.0, 1.0), swerveModule.translation2d)
    }


}