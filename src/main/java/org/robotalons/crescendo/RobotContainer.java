// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

// -------------------------------------------------------------[Robot Container]-----------------------------------------------------------//
/**
 *
 *
 * <h1>RobotContainer</h1>
 *
 * <p>Utility class which defines all modes of robot's event-cycle throughout it's lifetime.
 */
public final class RobotContainer {
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer INSTANCE = (null);

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {}

  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized RobotContainer getInstance() {
      if (java.util.Objects.isNull(INSTANCE)) {
          INSTANCE = new RobotContainer();
      }
      return INSTANCE;
  }
}
