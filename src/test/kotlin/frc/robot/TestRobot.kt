package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.sim.PhysicsSim

abstract class TestRobot: TimedRobot() {
    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }
    override fun simulationInit() {
    }
    override fun simulationPeriodic() {
        PhysicsSim.instance.run();
    }
}