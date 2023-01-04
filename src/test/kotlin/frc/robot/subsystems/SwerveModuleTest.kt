package frc.robot.subsystems


import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.sim.PhysicsSim
import frc.robot.subsystems.SwerveModuleTest.SwerveModuleSimulatedRobot
import frc.robot.util.SimulatedRobot
import frc.robot.util.SimulatedRobotTest
import frc.robot.util.SimulatedTest
import org.hamcrest.CoreMatchers.notNullValue
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assume
import org.junit.Test


/**
 * Test the swerve module.
 */
class SwerveModuleTest: SimulatedRobotTest<SwerveModuleSimulatedRobot>(
    SwerveModuleSimulatedRobot::class.java
) {

    val DELTA = 1e-2 // acceptable deviation range
    class SwerveModuleSimulatedRobot(test: SwerveModuleTest) : SimulatedRobot(test) {
        var module: SwerveModule = SwerveModule("test", 1, 2, 3, Translation2d(1.0, 1.0))
    }
    override var robot: SwerveModuleSimulatedRobot = SwerveModuleSimulatedRobot(this)
    var swerveModule: SwerveModule?
        get() = robot.module
        set(value) {
            if (value != null) {
                robot.module = value
            }
        }


    @After
    fun tearDown() {
        PhysicsSim.instance.reset()
    }

    @Test
    fun getDriveMotor() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertNotNull(swerveModule?.driveMotor)
    }

    @Test
    fun getSteerMotor() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertNotNull(swerveModule!!.steerMotor)
    }

    @Test
    fun getEnc() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertNotNull(swerveModule!!.enc)
    }

    @Test
    fun getAngle() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertEquals(0.0, swerveModule!!.angle, DELTA)
    }

    @Test
    fun getVelocity() {
        Assume.assumeThat(swerveModule, notNullValue())
        assertEquals(0.0, swerveModule!!.velocity, DELTA)
    }

    @Test
    @SimulatedTest
    fun move() {
        Assume.assumeThat(swerveModule, notNullValue())
        swerveModule!!.move(SwerveModuleState(1.1, Rotation2d(1.2)))
        HAL.observeUserProgramTeleop()
        for (i in 0..100) {
            // run the command scheduler and physics simulation
            CommandScheduler.getInstance().run()
            PhysicsSim.instance.run()
            Thread.sleep(20)
            HAL.observeUserProgramTeleop()
            // run the move method on the swerve module, passing in a drive value of 1.0 and an angle value of 1.0
            swerveModule!!.move(SwerveModuleState(1.1, Rotation2d(1.2)))
        }
        println("swerveModule!!.angle: " + swerveModule!!.angle)
        println("swerveModule!!.velocity: " + swerveModule!!.velocity)
        println("swerveModule!!.translation2d: " + swerveModule!!.translation2d)
        println(swerveModule!!.driveMotor.motorOutputPercent)
        assertEquals(1.0, swerveModule!!.velocity, DELTA)
        assertEquals(1.0, swerveModule!!.angle, DELTA)
    }

    @Test
    @SimulatedTest
    fun testMove() {
        // before running the test, initialize the HAL and observe the user program teleop
        HAL.observeUserProgramTeleop()
        // run the simulation and the swerve module's move method 100 times
        for (i in 0..100) {
            // run the command scheduler and physics simulation
            CommandScheduler.getInstance().run()
            PhysicsSim.instance.run()
            Thread.sleep(20)
            // run the move method on the swerve module, passing in a drive value of 1.0 and an angle value of 1.0
            swerveModule!!.move(1.0, 1.0)
        }
        // assert that the velocity and angle of the swerve module are as expected
        assertEquals(1.0, swerveModule!!.velocity, DELTA)
        assertEquals(1.0, swerveModule!!.angle, DELTA)
    }

    @Test
    fun zeroEncoders() {
        swerveModule!!.zeroEncoders()
        assertEquals(0.0, swerveModule!!.enc.position, DELTA)
    }

    @Test
    fun reset() {

        swerveModule!!.reset()
        assertEquals(0.0, swerveModule!!.angle, DELTA)
        assertEquals(0.0, swerveModule!!.velocity, DELTA)
    }

    @Test
    fun periodic() {
        swerveModule!!.periodic()
        assertEquals(0.0, swerveModule!!.angle, DELTA)
        assertEquals(0.0, swerveModule!!.velocity, DELTA)
    }

    @Test
    fun simulationPeriodic() {
        swerveModule!!.simulationPeriodic()
        assertEquals(0.0, swerveModule!!.angle, DELTA)
        assertEquals(0.0, swerveModule!!.velocity, DELTA)
    }

    @Test
    @SimulatedTest
    fun stop() {
        swerveModule!!.stop()
        Thread.sleep(100)
        assertEquals(0.0, swerveModule!!.velocity, DELTA)
    }

    @Test
    fun getTranslation2d() {
        assertEquals(Translation2d(1.0, 1.0), swerveModule!!.translation2d)
    }


}