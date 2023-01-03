package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.commands.DriveCommand
import frc.robot.commands.ResetCommand
import frc.robot.commands.ToggleBrakemodeCommand
import frc.robot.commands.ZeroEncodersCommand
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
open class RobotContainer(
    xbox: XboxController = XboxController(0)
) {
    var state: RobotState? = null

    val controlType: ControlScheme = DefaultControlScheme(xbox)

    val drivetrain = Drivetrain()

    var controlScheme = controlType.apply {
        // control scheme
        zeroEncoders.whileActiveOnce(ZeroEncodersCommand(drivetrain))
        toggleBrakeMode.whileActiveOnce(ToggleBrakemodeCommand(drivetrain))
        resetAll.whileActiveOnce(ResetCommand(drivetrain))

        // drive command
        forewardThresholdTrigger
            .or(strafeThresholdTrigger)
            .or(rotationThresholdTrigger)
            .whileActiveContinuous(RunCommand({
                DriveCommand(drivetrain, this).execute()
            }, drivetrain))
        forewardThresholdTrigger
            .or(strafeThresholdTrigger)
            .or(rotationThresholdTrigger)
            .negate()
            .whileActiveOnce(RunCommand({
                DriveCommand(drivetrain, 0.0, 0.0, 0.0).execute()
            }, drivetrain))
    }

    /**
     * called by robot periodic
     */
    open fun periodic() {
        // put the state in the smart dashboard along with control scheme data
        state?.let { state ->
            SmartDashboard.putString("state", state.name)
        }
        SmartDashboard.putNumber("forward", controlScheme.forward)
        SmartDashboard.putNumber("strafe", controlScheme.strafe)
        SmartDashboard.putNumber("rotation", controlScheme.rotation)
        SmartDashboard.putBoolean("brakeMode", drivetrain.brakeMode)
    }
    open fun disabledInit() {
        state = RobotState.DISABLED
        drivetrain.stop()
    }
    open fun disabledPeriodic() {}
    open fun autonomousInit() {
        state = RobotState.AUTONOMOUS
    }
    open fun autonomousPeriodic() {
    }
    open fun teleopInit() {
        state = RobotState.TELEOP
    }
    open fun teleopPeriodic() {
        controlScheme
    }
    open fun testInit() {
        state = RobotState.TEST
    }
    open fun testPeriodic() {}
    open fun simulationPeriodic() {
    }

    open fun robotInit() {}
}