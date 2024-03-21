// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.utilities;
import edu.wpi.first.hal.util.BoundaryException;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;

import java.util.Objects;

import javax.validation.constraints.NotNull;

import com.jcabi.aspects.Immutable.Array;

// --------------------------------------------------------------[Type Vector]--------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Vector</h1>
 * 
 * <p>Simple utility wrapper class for an immutable generic array of elements with a pre-defined size.<p>
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class TypeVector<@NotNull Type, @NotNull Elements extends Num> {
  // -------------------------------------------------------------[Constants]---------------------------------------------------------------//
  private transient final @NotNull @Array Type[] VECTOR;
  private transient final @NotNull Nat<Elements> ELEMENTS;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Type Vector Constructor.
   * @param Elements              Natural number representation of the number of elements in this array
   * @param Vector                Array data to place within the bounds of the Vector's array, length should match the number of elements specified.
   * @throws BoundaryException    Bounds of the array are exceeded or not met by the length of the Vector parameter
   * @throws NullPointerException Either the Elements parameter or the Vector parameter evaluate to null 
   */
  @SuppressWarnings("unchecked")
  public TypeVector(final Nat<Elements> Elements, final Type... Vector) throws BoundaryException, NullPointerException {
    try {
      assert Objects.requireNonNull(Elements.getNum()) == Objects.requireNonNull(Vector).length;
    } catch(final AssertionError Ignored) {
      throw new BoundaryException(
        String.format(
          ("Bounds of TypeVector defined [%d] do not match the length of the vector [%d] excepted"), 
          Elements.getNum(), 
          Vector.length));
    }
    VECTOR = Objects.requireNonNull(Vector);
    ELEMENTS = Objects.requireNonNull(Elements);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Provides the value of a specific point within the array
   * @param Index Point within the array to get a value from
   * @see #getVector()
   */
  public synchronized Type get(final Integer Index) {
    return VECTOR[Index];
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the underlying, specified type, array that was defined during construction, will always meet the expected bounds of Elements.
   * @return Array of specified type
   * @see #get(Integer)
   */
  public Type[] getVector() {
    return VECTOR;
  }
}
