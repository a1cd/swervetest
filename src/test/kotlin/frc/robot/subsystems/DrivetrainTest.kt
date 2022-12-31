@file:Suppress(
    "UnusedImport",
    "unused",
    "RemoveEmptyClassBody",
    "RedundantSemicolon",
)
package frc.robot.subsystems
// --- DRIVETRAIN TEST ---
//
// This is a test for the Drivetrain subsystem.
// This will test
// - the drivetrain subsystem's move() function.
//   - correctly tells the modules how fast to drive each module
//   - correctly tells the modules how much to turn each module
// - the drivetrain subsystem's stop() function.
//   - must call each swerve module's stop() function
// - the zero encoders function
//   - must call each swerve module's zero encoder function
// - the reset odometry function
//   - must call each swerve module's reset odometry function
// - the brake mode variable
//   - must check each swerve module's brake mode variable
//   - should only be true if all swerve modules are in brake mode
//   - setting brake mode should set all swerve modules' brake mode

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.Constants
import org.junit.jupiter.api.BeforeEach

/**
 * Test the drivetrain subsystem.
 */
class DrivetrainTest {
    class TestSwerveModule: SwerveModule {
        var moveCalled: Pair<Double, Double>? = null
        var stopCalled: Int = 0
        var zeroEncoderCalled: Int = 0
        var resetCalled: Int = 0
        var brakeModeSetValue: Boolean? = null
        var brakeModeGet: Int = 0
        var brakeModeSet: Int = 0
        override fun move(drive: Double, angle: Double) {
            super.move(drive, angle)
            moveCalled = Pair(drive, angle)
        }
        override fun move(state: SwerveModuleState) {
            super.move(state)
            moveCalled = Pair(state.speedMetersPerSecond, state.angle.radians)
        }
        override fun stop() {
            super.stop()
            stopCalled += 1
        }
        override fun zeroEncoders() {
            super.zeroEncoders()
            zeroEncoderCalled += 1
        }
        override fun reset() {
            super.reset()
            resetCalled += 1
        }
        override var brakeMode: Boolean
            get() {
                brakeModeGet += 1
                return super.brakeMode
            }
            set(value) {
                brakeModeSet += 1
                super.brakeMode = value
            }

        /**
         * construct from a swerve module
         */
        constructor(module: SwerveModule): super(
            module.moduleName,
            module.driveId,
            module.steerId,
            module.encId,
            module.translation2d
        )
    }
    val fl = TestSwerveModule(SwerveModule("fl", Constants.FrontLeftDriveMotor, Constants.FrontLeftSteerMotor, Constants.FrontLeftEncoder, Translation2d(1.0, 1.0)))
    val fr = TestSwerveModule(SwerveModule("fr", Constants.FrontRightDriveMotor, Constants.FrontRightSteerMotor, Constants.FrontRightEncoder, Translation2d(1.0, 1.0)))
    val bl = TestSwerveModule(SwerveModule("bl", Constants.BackLeftDriveMotor, Constants.BackLeftSteerMotor, Constants.BackLeftEncoder, Translation2d(1.0, 1.0)))
    val br = TestSwerveModule(SwerveModule("br", Constants.BackRightDriveMotor, Constants.BackRightSteerMotor, Constants.BackRightEncoder, Translation2d(1.0, 1.0)))
    val drivetrain = Drivetrain(fl, fr, bl, br)
    @BeforeEach
    fun beforeEach() {

    }


}