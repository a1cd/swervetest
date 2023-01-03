package frc.robot.subsystems


import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.TestRobot
import frc.robot.sim.PhysicsSim
import org.hamcrest.CoreMatchers.notNullValue
import org.junit.*
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.rules.Timeout
import java.util.concurrent.TimeUnit


/**
 * Test the swerve module.
 */
class SwerveModuleTest {
    @Rule
    @JvmField
    var timeout: Timeout = Timeout(10, TimeUnit.SECONDS)

    val DELTA = 1e-2 // acceptable deviation range


    class SwerveModuleTestRobot: TestRobot() {
        var module: SwerveModule = SwerveModule("test", 1, 2, 3, Translation2d(0.0, 0.0))
        override fun robotInit() {
            super.robotInit()
            module.move(1.0, 1.0)
        }
    }
    var robot: SwerveModuleTestRobot? = null
    //get/setter
    var swerveModule: SwerveModule?
        get() = robot?.module
        set(value) {
            if (value != null) {
                robot?.module = value
            }
        }

    @Before
    fun setUp() {
        // start the robot
        RobotBase.startRobot { SwerveModuleTestRobot().also { robot = it } }
    }


    @After
    fun tearDown() {
        robot?.close()
        robot = null
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
        assertEquals(0.0, swerveModule!!.enc.position % 360.0, DELTA)
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
    fun stop() {
        swerveModule!!.stop()
        assertEquals(0.0, swerveModule!!.velocity, DELTA)
    }

    @Test
    fun getTranslation2d() {
        assertEquals(Translation2d(1.0, 1.0), swerveModule!!.translation2d)
    }


}