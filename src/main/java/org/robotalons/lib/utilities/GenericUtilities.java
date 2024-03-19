// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

// ------------------------------------------------------------[GenericUtilities]--------------------------------------------------------------//
/**
 * 
 * 
 * <h1>MathUtilities</h1>
 * 
 * <p>Simple generic utility helper functionality, provides simple static methods for doing simple, but repetitive methods
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class GenericUtilities {

  /**
   * Provides a safe environment for executing possibly null operations
   * @param Executable Runnable that executes a possibly null-pointer operation.
   */
  public static void protect(final Runnable Executable) {
    try {
      Executable.run();
    } catch (final NullPointerException Ignored) {}
  }
}
