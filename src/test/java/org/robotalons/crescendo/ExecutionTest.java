// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
import org.junit.jupiter.api.Test;

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
   //TODO: Enable Teleop During Testing
    Executor.shutdown();
    assertDoesNotThrow(() -> {
      try {
        Task.get(TEST_DURATION, TimeUnit.MILLISECONDS);
        Executor.shutdownNow();
      } catch(final CancellationException | SecurityException | TimeoutException Ignored) {}
    });
    
  }
}
