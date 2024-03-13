// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.inputs.LoggedDriverStation.DriverStationInputs;

import java.util.concurrent.CancellationException;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
// -------------------------------------------------------------[Execution Test]-----------------------------------------------------------//
public final class ExecutionTest {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final Long TEST_DURATION = (5000L);
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Instantiates a new robot, asserts that no errors are thrown within the alloted time limit.
   */
  @Test
  synchronized void CheckErrorMain() {
    final var Executor = Executors.newFixedThreadPool((1));
    final var Task = Executor.submit(() -> {
      org.robotalons.crescendo.Main.main();
    });
    assertDoesNotThrow(() -> {
      try {
        final var Field = LoggedDriverStation.class.getField(("dsInputs"));
        final var Faked = new DriverStationInputs();
        Faked.enabled = (true);
        Field.setAccessible((true));
        Field.set((null), Faked);
        Task.get(TEST_DURATION, TimeUnit.MILLISECONDS);
        Executor.shutdownNow();
      } catch(final CancellationException | TimeoutException | SecurityException | NoSuchFieldException | IllegalAccessException Ignored) {}
    });
    
  }
}
