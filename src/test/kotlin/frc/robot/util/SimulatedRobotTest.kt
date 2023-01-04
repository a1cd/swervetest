package frc.robot.util

import edu.wpi.first.wpilibj.RobotBase
import org.junit.Rule
import org.junit.rules.Timeout
import org.junit.runners.model.FrameworkMethod
import org.junit.runners.model.Statement
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * annotation for test that is simulated
 */
@Target(AnnotationTarget.FUNCTION)
@Retention(AnnotationRetention.RUNTIME)
annotation class SimulatedTest
// only run setup before tests that are annotated with @SimulatedTest
class SimulatedTestRule: org.junit.rules.MethodRule {
    override fun apply(base: Statement?, method: FrameworkMethod?, target: Any?): Statement? {
        if (method != null) {
            if (method.getAnnotation(SimulatedTest::class.java) != null) {// if the test is annotated with @SimulatedTest
                return object : Statement() {// create a new statement
                    override fun evaluate() {// evaluate the statement
                        // run setup
                        // if class that annotated test is in inherits SimulatedRobotTest
                    // run setup
                    if (target is SimulatedRobotTest<*>) {
                        target.setup()
                    }
                        base?.evaluate()// run the test
                    }
                }
            }
        }
        return base
    }
}

/**
 * takes a class that extends SimulatedRobot and runs it before each test that is annotated with @SimulatedTest
 */
abstract class SimulatedRobotTest<T: SimulatedRobot>(
    private var robotClass: Class<out T>
) {
    @Rule
    @JvmField
    var timeout: Timeout = Timeout(30, TimeUnit.SECONDS)
    // only run if test is annotated with SimulatedTest
    @Rule
    @JvmField
    var simulatedTestRule: SimulatedTestRule = SimulatedTestRule()
    var robotInitLatch: CountDownLatch? = null
    abstract var robot: T
    fun setup() {
        robotInitLatch = CountDownLatch(1)
        val latch = CountDownLatch(1)
        Thread {
            RobotBase.startRobot { //FIXME: Concurent modification exception!!
                robotClass.getConstructor(this.javaClass).newInstance(this@SimulatedRobotTest).also {
                    robot = it
                    latch.countDown()
                }
            }
        }.start()
        latch.await()
        robotInitLatch!!.await()
    }
}