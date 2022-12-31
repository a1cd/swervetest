package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

/**
 * Toggle the brake mode of the drivetrain
 */
class ToggleBrakemodeCommand(
    private val drivetrain: Drivetrain
): CommandBase() {
    init {
        addRequirements(drivetrain)
    }
    override fun execute() {
        drivetrain.brakeMode = !drivetrain.brakeMode
    }
    override fun isFinished(): Boolean = true
}