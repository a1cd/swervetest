package frc.robot.subsystems


import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.hal.HAL
import edu.wpi.first.hal.HALUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.Constants
import frc.robot.sim.PhysicsSim
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Disabled
import org.junit.jupiter.api.Test


class SwerveModuleTest {

    val DELTA = 1e-2 // acceptable deviation range

    var swerveModule: SwerveModule? = null
    var simEncoder: EncoderSim? = null
    @BeforeEach
    fun setUp() {
        // initialize the HAL to start the simulation.
        // pass in 500 as the timeout in milliseconds and 0 as the device ID
        assert(HAL.initialize(500, 0))
        SimHooks.setHALRuntimeType(HALUtil.getHALRuntimeType())
        // create our simulation encoder at the correct CAN index
        simEncoder = EncoderSim.createForIndex(Constants.FrontLeftEncoder)
        // create our swerve module using the appropriate motor and encoder IDs and a Translation2d object
        swerveModule = SwerveModule(
            "fl",
            Constants.FrontLeftDriveMotor,
            Constants.FrontLeftSteerMotor,
            Constants.FrontLeftEncoder,
            Translation2d(1.0, 1.0)
        )
    }


    @AfterEach
    fun tearDown() {
        swerveModule = null
        PhysicsSim.instance.reset()
    }

    @Test
    fun getDriveMotor() {
        assertNotNull(Constants.FrontLeftDriveMotor)
    }

    @Test
    fun getSteerMotor() {
        assertNotNull(swerveModule!!.steerMotor)
    }

    @Test
    fun getEnc() {
        assertNotNull(swerveModule!!.enc)
    }

    @Test
    fun getAngle() {
        assertEquals(0.0, swerveModule!!.angle, DELTA)
    }

    @Test
    fun getVelocity() {
        assertEquals(0.0, swerveModule!!.velocity, DELTA)
    }

    @Test
    @Disabled
    fun move() {
        swerveModule!!.move(SwerveModuleState(1.0, Rotation2d(1.0)))
        assertEquals(1.0, swerveModule!!.velocity, DELTA)
        assertEquals(1.0, swerveModule!!.angle, DELTA)
    }

    @Test
    fun testMove() {
        // before running the test, initialize the HAL and observe the user program teleop
        HAL.initialize(500, 0)
        HAL.observeUserProgramTeleop()
        // run the simulation and the swerve module's move method 100 times
        for (i in 0..100) {
            swerveModule!!.driveMotor.set(ControlMode.PercentOutput, 1.0)
            // run the command scheduler and physics simulation
            CommandScheduler.getInstance().run()
            swerveModule!!.driveMotor.set(ControlMode.PercentOutput, 1.0)
            PhysicsSim.instance.run()
            swerveModule!!.driveMotor.set(ControlMode.PercentOutput, 1.0)
            // run the move method on the swerve module, passing in a drive value of 1.0 and an angle value of 1.0
            swerveModule!!.move(1.0, 1.0)
        }
        print(swerveModule)
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