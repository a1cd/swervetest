package frc.robot.util

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.sim.PhysicsSim
open class SimulatedRobot(
    private var test: SimulatedRobotTest<out SimulatedRobot>
): TimedRobot() {
    override fun robotInit() {
        super.robotInit()
        this.test.robotInitLatch?.countDown()
    }
    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }
    override fun simulationPeriodic() {
        PhysicsSim.instance.run()
    }
}