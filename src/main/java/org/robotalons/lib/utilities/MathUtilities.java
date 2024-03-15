// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

// -------------------------------------------------------------[MathUtilities]--------------------------------------------------------------//
/**
 * 
 * 
 * <h1>MathUtilities</h1>
 * 
 * <p>Simple math utility helper functionality, provides simple static methods for doing simple, but repetitive calculations; add methods as needed.
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class MathUtilities {

  /**
   * Provides the standard deviation for a given set of numbers 
   * @param Numbers Collection (array) of data to find the standard deviation of
   * @return Standard deviation as a double value
   */
  public static double standardDeviation(double[] Numbers){
    double Mean = 0d, SummativeSquareDifference = 0d;
    for(double Number : Numbers){
      Mean += Number;
    }
    Mean /= Numbers.length;
    for(double Number : Numbers){
      SummativeSquareDifference += Math.pow((Number - Mean), (2));
    }
    return Math.sqrt(SummativeSquareDifference / Numbers.length - 1);
  }

  /**
   * Applies squared inputs to a given input, while retaining the sign
   * @param Input Any Real Number
   * @return Input Squared, with the same sign of the original
   */
  public static double signedSquare(final double Input) {
    return Math.copySign(Input * Input, Input);
  }
}
