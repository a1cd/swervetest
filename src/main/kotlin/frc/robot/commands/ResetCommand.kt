package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class ResetCommand(
    private val drivetrain: Drivetrain
): CommandBase() {
    init {
        addRequirements(drivetrain)
    }
    override fun execute() {
        drivetrain.reset()
    }
    override fun isFinished(): Boolean = true
}