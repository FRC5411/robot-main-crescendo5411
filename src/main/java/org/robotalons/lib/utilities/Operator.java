// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.mockito.DoNotMock;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
// ----------------------------------------------------------------[Operator]--------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Operator</h1>
 * 
 * <p> Represents a robot driver's personalized preferences and keybindings that can be used to control the robot to their specifications,
 * and is parametrized with an enum containing the necessary mappings for keybindings and preferences.
 * 
 * @author Cody Washington (@Jelatinone) 
 */
@DoNotMock
public final class Operator<Keybindings extends Enum<?>, Preferences extends Enum<?>> implements Sendable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<Operator<?,?>> OPERATORS = new ArrayList<>();
  private final Map<Preferences,Supplier<?>> PREFERENCES = new HashMap<>();  
  private final Map<Keybindings,Trigger> KEYBINDINGS = new HashMap<>();
  //TODO: Controller Status
  private final String PILOT_NAME;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private volatile SendableBuilder Builder;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Constructor.
   * @param Name Pilot's name to reference against other pilot profiles
   */
  public Operator(final String Name) {
    PILOT_NAME = Name;
    OPERATORS.add(this);
  }  
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public void initSendable(final SendableBuilder Builder) {
    this.Builder = Builder;
    Builder.addStringProperty(PILOT_NAME, this::getName, (String) -> {});
    PREFERENCES.forEach((Attribute, Value) -> Builder.addStringProperty(Attribute.toString(), Value::toString, (String) -> {}));
    KEYBINDINGS.forEach((Attribute, Value) -> Builder.addStringProperty(Attribute.toString(), Value::toString, (String) -> {}));
    Builder.update();
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Adds a new keybinding to the keybinding(s) map of this operator
   * @param Keybinding Enum type value that can be added to the map
   * @param Trigger    Valid trigger of this Operator's GenericHID input
   * @return This operator, for chained calls to {@link #add(Enum, Supplier)} or {@link #add(Enum, Trigger)}
   */
  public Operator<Keybindings,Preferences> add(final Keybindings Keybinding, final Trigger Trigger) {
    KEYBINDINGS.put(Keybinding, Trigger);
    if(Builder != (null)) {
      Builder.addStringProperty(Keybinding.toString(), Trigger::toString, (String) -> {});
      Builder.update();
    }
    return this;
  }

  /**
   * Adds a new keybinding to the preference(s) map of this operator
   * @param Preference Enum type value that can be added to the map
   * @param Value      Valid supplier of a changing preference of this operator
   * @return This operator, for chained calls to {@link #add(Enum, Supplier)} or {@link #add(Enum, Trigger)}
   */
  public <Supplied> Operator<Keybindings,Preferences> add(final Preferences Preference, final Supplier<Supplied> Value) {
    PREFERENCES.put(Preference, Value);
    if(Builder != (null)) {
      Builder.addStringProperty(Preference.toString(), () -> Value.get().toString(), (String) -> {});
      Builder.update();
    }
    return this;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the value of a given keybinding from this operator's keybinding map
   * @param Keybinding Enum type value to pull information from
   * @return The keybinding if it exists within the map, if not {@code NullPointerException} is thrown
   */
  public Trigger getKeybinding(final Keybindings Keybinding) {
    return KEYBINDINGS.get(Keybinding);
  }

  /**
   * Provides the value of a given preference from this operator's keybinding map
   * @param Preference Enum type value to pull information from
   * @return The value of preference's supplier if it exists within the map, if not {@code NullPointerException} is thrown
   */
  public Object getPreference(final Preferences Preference) {
    return PREFERENCES.get(Preference).get();
  }

  /**
   * Provides a list of all constructed operator objects
   * @return List of Operators
   */
  public static List<Operator<?,?>> getOperators() {
    return OPERATORS;
  }

  /**
   * Retrieves the pilot's name
   * @return Name of pilot
   */
  public String getName() {
    return PILOT_NAME;
  }
}