package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.simulation.XboxControllerSim
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.RunCommand
import org.junit.jupiter.api.*
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Assumptions.assumeFalse
import org.junit.jupiter.api.Assumptions.assumeTrue

/**
 * Test the robot class.
 */
class RobotTest {
    // robot
    private var robot: Robot? = null

    // xboxSim controller
    private var xboxSim: XboxControllerSim? = null
    private var xboxController: XboxController? = null


    // simulation
    class TestRobotContainer(xbox: XboxController) : RobotContainer(xbox) {
        var lastCommand: String? = null
        override fun periodic() {
            lastCommand = "periodic"
        }
        override fun disabledInit() {
            lastCommand = "disabledInit"
        }
        override fun disabledPeriodic() {
            lastCommand = "disabledPeriodic"
        }
        override fun autonomousInit() {
            lastCommand = "autonomousInit"
        }
        override fun autonomousPeriodic() {
            lastCommand = "autonomousPeriodic"
        }
        override fun teleopInit() {
            lastCommand = "teleopInit"
        }
        override fun teleopPeriodic() {
            lastCommand = "teleopPeriodic"
        }
        override fun testInit() {
            lastCommand = "testInit"
        }
        override fun testPeriodic() {
            lastCommand = "testPeriodic"
        }
        override fun simulationPeriodic() {
            lastCommand = "simulationPeriodic"
        }
        override fun robotInit() {
            lastCommand = "robotInit"
        }
    }
    private var testRobotContainer: TestRobotContainer? = null
    @BeforeEach
    fun setUp() {
        // setup robot simulation
        robot = Robot()
        // setup xboxSim controller simulation
        xboxSim = XboxControllerSim(0)
        xboxController = XboxController(0)
        // setup robot container
        testRobotContainer = TestRobotContainer(xboxController!!)
    }
    @AfterEach
    fun tearDown() {
        // stop robot simulation
        this.robot!!.disabledInit()
        this.robot = null
        // stop xboxSim controller simulation
        this.xboxSim = null
        this.xboxController = null
        // stop robot container
        this.testRobotContainer = null
    }
    @Test
    @DisplayName("test that robot container is initialized")
    fun robotInit() {
        Assumptions.assumingThat(this.robot!!.robotContainer==null) {
            // test robot init
            this.robot!!.robotInit()
            assertNotNull(this.robot!!.robotContainer)
        }
    }
    @Test
    @DisplayName("test robot periodic - check that robot advances command scheduler")
    fun robotPeriodic() {
        // -- before --
        // test command scheduler using a command and time since it was scheduled
        class TestCommand: RunCommand({}) {
            var hasExecuted = false
            override fun execute() {
                super.execute()
                this.hasExecuted = true
            }
            override fun isFinished(): Boolean = false
        }
        CommandScheduler.getInstance().cancelAll()
        val testCommand = TestCommand().apply {
            CommandScheduler.getInstance().schedule(this)
        }
        assumeTrue(CommandScheduler.getInstance().timeSinceScheduled(testCommand) == 0.0)
        // test robot periodic
        this.robot!!.robotPeriodic()

        // -- after --
        assertTrue(CommandScheduler.getInstance().timeSinceScheduled(testCommand) > 0.0001)
        assertTrue(testCommand.hasExecuted)
    }
    @Test
    @DisplayName("test motor power is zero when disabled")
    fun disabledInit() {
        // advance simulation time
        robot!!.robotInit()
        robot!!.robotPeriodic()

        // switch sim robot to test mode
        robot!!.testInit()
        robot!!.testPeriodic()

        // make sure motor
        assumeFalse(
            robot!!.robotContainer == null,
            "robot container is null"
        )
        // motor speed is zero
        assumeTrue(
            0.0==robot!!.robotContainer!!.drivetrain.fl.driveMotor.selectedSensorVelocity * 10.0 / 4096.0,
            "motor speed should be 0.0 before test"
        )

        // move the robot using the test control scheme
        xboxSim!!.setRawAxis(0, 0.5)
        xboxSim!!.setRawAxis(1, 0.5)

        // advance the simulation
        for (i in 0..5) {
            robot!!.robotPeriodic()
        }

        // check motor percent outputs are not zero with some tolerance
        assumeFalse(
            robot!!.robotContainer == null,
            "robot container is null"
        )
        assumeFalse(
            0.01 > Math.abs(robot!!.robotContainer!!.drivetrain.fl.driveMotor.selectedSensorVelocity * 10.0 / 4096.0),
            "motor speed should increase after moving xbox joystick"
        )

        // finally actually test to see if the robot properly disables
        // test robot disabled init
        robot!!.disabledInit()

        // advance the simulation
        for (i in 0..15) {
            robot!!.robotPeriodic()
            robot!!.disabledPeriodic()
        }
        assumeFalse(
            robot!!.robotContainer==null,
            "robot container is null"
        )
        assertEquals(
            0.0,
            robot!!.robotContainer!!.drivetrain.fl.driveMotor.selectedSensorVelocity * 10.0 / 4096.0,
            0.01,
            "motor power should be zero after disabling"
        )
    }
    @Test
    @DisplayName("Test robot container disabled init is called when robot is disabled")
    fun disabledInitSubcallTest() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.disabledInit()
        assertEquals(
            "disabledInit",
            testRobotContainer!!.lastCommand,
            "disabledInit() should call robotContainer.disabledInit()"
        )
    }
    @Test
    @DisplayName("Test robot container disabled periodic is called when robot is disabled")
    fun disabledPeriodic() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.disabledPeriodic()
        assertEquals(
            "disabledPeriodic",
            testRobotContainer!!.lastCommand,
            "disabledPeriodic() should call robotContainer.disabledPeriodic()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.autonomousInit() called when autonomousInit() is called")
    fun testInit() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.testInit()
        assertEquals(
            "testInit",
            testRobotContainer!!.lastCommand,
            "testInit() should call robotContainer.testInit()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.testPeriodic() called when testPeriodic() is called")
    fun testPeriodic() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.testPeriodic()
        assertEquals(
            "testPeriodic",
            testRobotContainer!!.lastCommand,
            "testPeriodic() should call robotContainer.testPeriodic()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.autonomousInit() called when autonomousInit() is called")
    fun autonomousInit() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.autonomousInit()
        assertEquals(
            "autonomousInit",
            testRobotContainer!!.lastCommand,
            "autonomousInit() should call robotContainer.autonomousInit()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.autonomousPeriodic() called when autonomousPeriodic() is called")
    fun autonomousPeriodic() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.autonomousPeriodic()
        assertEquals(
            "autonomousPeriodic",
            testRobotContainer!!.lastCommand,
            "autonomousPeriodic() should call robotContainer.autonomousPeriodic()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.teleopInit() called when teleopInit() is called")
    fun teleopInit() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.teleopInit()
        assertEquals(
            "teleopInit",
            testRobotContainer!!.lastCommand,
            "teleopInit() should call robotContainer.teleopInit()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.teleopPeriodic() called when teleopPeriodic() is called")
    fun teleopPeriodic() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.teleopPeriodic()
        assertEquals(
            "teleopPeriodic",
            testRobotContainer!!.lastCommand,
            "teleopPeriodic() should call robotContainer.teleopPeriodic()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.robotPeriodic() called when robotPeriodic() is called")
    fun simulationPeriodic() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.simulationPeriodic()
        assertEquals(
            "simulationPeriodic",
            testRobotContainer!!.lastCommand,
            "simulationPeriodic() should call robotContainer.simulationPeriodic()"
        )
    }
    @Test
    @DisplayName("Test robotContainer.robotPeriodic() called when robotPeriodic() is called")
    fun simulationInit() {
        this.robot!!.robotContainer = testRobotContainer
        this.robot!!.simulationInit()
        assertEquals(
            RobotContainer(xboxController!!).javaClass,
            robot!!.robotContainer?.javaClass,
            "simulationInit() should have set robotContainer to the normal robotContainer."
        )
    }
}
