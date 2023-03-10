package frc.robot.subsystems


import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.PWMSim
import frc.robot.Constants
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test


class SwerveModuleTest {

    val DELTA = 1e-2 // acceptable deviation range

    var swerveModule: SwerveModule? = null
    var simMotorDrive: PWMSim? = null
    var simMotorAngle: PWMSim? = null
    var simEncoder: EncoderSim? = null

    @BeforeEach
    fun setUp() {
        assert(
            HAL.initialize(500, 0) // initialize the HAL, crash if failed
        )

        simMotorDrive = PWMSim(Constants.FrontLeftDriveMotor) // create our simulation PWM motor controller
        simMotorAngle = PWMSim(Constants.FrontLeftSteerMotor) // create our simulation PWM motor controller
        // create our simulation encoder at correct CAN index
        simEncoder = EncoderSim.createForIndex(Constants.FrontLeftEncoder)
        swerveModule = SwerveModule(
            "fl",
            Constants.FrontLeftDriveMotor,
            Constants.FrontLeftSteerMotor,
            Constants.FrontLeftEncoder,
            Translation2d(1.0, 1.0)
        ) // create our swerve module




    }

    @AfterEach
    fun tearDown() {
        swerveModule = null
        simMotorDrive = null
        simMotorAngle = null
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
    fun move() {
        swerveModule!!.move(SwerveModuleState(1.0, Rotation2d(1.0)))
        assertEquals(1.0, swerveModule!!.velocity, DELTA)
        assertEquals(1.0, swerveModule!!.angle, DELTA)
    }

    @Test
    fun testMove() {
        swerveModule!!.move(1.0, 1.0)
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