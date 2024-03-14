// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.inputs.LoggedDriverStation.DriverStationInputs;

import java.util.concurrent.CancellationException;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
// -------------------------------------------------------------[Execution Test]-----------------------------------------------------------//
public final class ExecutionTest {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final Long TEST_DURATION = (5000L);
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Instantiates a new robot, asserts that no errors are thrown within the alloted time limit.
   */
  @Test
  synchronized void checkErrorMain() throws Exception {
    final var Executor = Executors.newFixedThreadPool((1));
    Executor.submit(() -> Main.main());
    TimeUnit.MILLISECONDS.sleep(TEST_DURATION);
    final var Field = LoggedDriverStation.class.getField(("dsInputs"));
    final var Faked = new DriverStationInputs();
    Executor.wait();
    Faked.enabled = (true);
    Field.setAccessible((true));
    Field.set((null), Faked);
    Executor.notifyAll();
    TimeUnit.MILLISECONDS.sleep(TEST_DURATION);
    Executor.shutdownNow();    
  }
}
