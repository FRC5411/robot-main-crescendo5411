// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
// ----------------------------------------------------------------[Operator]--------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Operator</h1>
 * 
 * <p> Represents a robot driver's personalized preferences and keybindings that can be used to control the robot to their desires. </p>
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public final class Operator implements Sendable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Map<String,Supplier<Object>> PREFERENCES = new HashMap<>();  
  private final Map<String,Trigger> KEYBINDINGS = new HashMap<>();
  private final String PILOT_NAME;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private SendableBuilder CurrentBuilder;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Constructor.
   * @param Name Pilot's name to reference against other pilot profiles
   */
  public Operator(final String Name) {
    PILOT_NAME = Name;
  }  
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public void initSendable(final SendableBuilder Builder) {
    CurrentBuilder = Builder;
    Builder.addStringProperty(PILOT_NAME, this::getName, (String) -> {});
    PREFERENCES.forEach((Attribute, Value) -> Builder.addStringProperty(Attribute, Value::toString, (String) -> {}));
    KEYBINDINGS.forEach((Attribute, Value) -> Builder.addStringProperty(Attribute, Value::toString, (String) -> {}));
    Builder.update();
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Adds a new keybinding to the pilot's keybinding mapping
   * @param Keybinding Name of keybinding
   * @param Trigger    Keybinding Trigger
   */
  public Operator addKeybinding(final String Keybinding, final Trigger Trigger) {
    KEYBINDINGS.put(Keybinding, Trigger);
    if(CurrentBuilder != null) {
      CurrentBuilder.addStringProperty(Keybinding, Trigger::toString, (String) -> {});
      CurrentBuilder.update();
    }
    return this;
  }
  /**
   * Add a new preference to the pilot's preference mapping
   * @param Preference Name of preference
   * @param Value Value of preference, can be any object
   */
  public Operator addPreference(final String Preference, final Supplier<Object> Value) {
    PREFERENCES.put(Preference, Value);
    if(CurrentBuilder != null) {
      CurrentBuilder.addStringProperty(Preference, () -> Value.get().toString(), (String) -> {});
      CurrentBuilder.update();
    }
    return this;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves a keybinding from the pilot's keybinding mapping
   * @param Keybinding Name of keybinding
   * @return A trigger from the pilot's keybinding map
   */
  public Trigger getKeybinding(final String Keybinding) {
    return KEYBINDINGS.get(Keybinding);
  }
  /**
   * Retrieves a preference from the pilot's preference mapping
   * @param Preference Name of preference
   * @return An object from the pilot's keybinding map
   */
  public Object getPreference(final String Preference) {
    return PREFERENCES.get(Preference).get();
  }

  /**
   * Retrieves the pilot's name
   * @return Name of pilot
   */
  public String getName() {
    return PILOT_NAME;
  }
}