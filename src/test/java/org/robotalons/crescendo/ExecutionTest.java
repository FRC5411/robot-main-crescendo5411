// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
// -------------------------------------------------------------[Execution Test]-----------------------------------------------------------//
public final class ExecutionTest {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final Long TEST_RUNTIME_DURATION = (5_000L);
  public static final Long TEST_START_DURATION = (7_500L);
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Test
  synchronized void checkErrorRuntime() throws Exception {
    final var RuntimeExecutor = Executors.newFixedThreadPool((1));
    RuntimeExecutor.submit(() -> Main.main());
    TimeUnit.MILLISECONDS.sleep(TEST_START_DURATION);
    RuntimeExecutor.execute(() -> {
      try {
        RuntimeExecutor.wait();
      } catch(final Exception Ignored) {}
    });
    final var RuntimeInputs = LoggedDriverStation.class.getDeclaredField(("dsInputs"));
    RuntimeInputs.setAccessible((true));
    final var RuntimeEnabled = LoggedDriverStation.getDSData().getClass().getDeclaredField(("enabled"));
    RuntimeEnabled.setAccessible((true));
    RuntimeEnabled.setBoolean(LoggedDriverStation.getDSData(), (true));
    RuntimeExecutor.execute(() -> {
      try {
        RuntimeExecutor.notify();
      } catch(final Exception Ignored) {}
    });
    TimeUnit.MILLISECONDS.sleep(TEST_RUNTIME_DURATION);
    RuntimeExecutor.shutdownNow();
  }
}
