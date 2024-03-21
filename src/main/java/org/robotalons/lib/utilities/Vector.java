// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;

import java.util.Objects;

// ----------------------------------------------------------------[Vector]--------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Vector</h1>
 * 
 * <p>Simple utility wrapper class for a generic array of elements with a pre-defined size.<p>
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class Vector<Type, Elements extends Num> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public final Type[] DATA;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Vector constructor.
   * @param Elements Number of a given type of elements this array stores
   * @param Data     Data stored within the array, size of which must match the size specified by Elements
   * @throws BoundaryException When the number of Elements does not match the size of data 
   */
  @SuppressWarnings("unchecked")
  public Vector(final Nat<Num> Elements, final Type... Data) throws BoundaryException {
    if(Objects.requireNonNull(Elements).getNum() == Data.length) {
      DATA = Objects.requireNonNull(Data);
    } else {
      throw new BoundaryException(("The size of the provided data must match the declared size when creating a new vector"));
    }
  }
}
