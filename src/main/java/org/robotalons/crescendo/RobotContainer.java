// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.SubsystemManager;
import org.robotalons.lib.utilities.Alert;
import org.robotalons.lib.utilities.Alert.AlertType;
import org.robotalons.lib.utilities.Operator;

import java.util.ArrayList;
import java.util.List;
// -------------------------------------------------------------[Robot Container]-----------------------------------------------------------//
/**
 *
 *
 * <h1>RobotContainer</h1>
 *
 * <p>Utility class which defines all modes of robot's event-cycle throughout it's lifetime.
 */
public final class RobotContainer {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final List<LoggedDashboardChooser<Operator<Keybindings, Preferences>>> OperatorSelectors;
  public static LoggedDashboardChooser<Command> AutonomousSelector;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // private static DigitalInput beamBreak1 = new DigitalInput(1);
  // private static DigitalInput beamBreak2 = new DigitalInput(2);

  //LEDStrip STRIP = new LEDStrip(26, 0);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  
  private RobotContainer() {} static {
    OperatorSelectors = new ArrayList<>();
    SubsystemManager.getSubsystems().forEach((Subsystem) -> {
      final var Selector = new SendableChooser<Operator<Keybindings, Preferences>>();
      final var Default = Profiles.DEFAULT.get(Subsystem);
      Profiles.OPERATORS.forEach((Profile) -> Selector.addOption(Profile.getName(), Profile));
      Selector.setDefaultOption(Default.getName(), Default);
      Selector.onChange(Subsystem::configure);
      Subsystem.configure(Default);
      OperatorSelectors.add(new LoggedDashboardChooser<Operator<Keybindings, Preferences>>(Subsystem.getName() + " Pilot Selector", Selector));
    });
    Profiles.OPERATORS.forEach((Profile) -> SmartDashboard.putData(Profile.getName(), Profile));
    SubsystemManager.getInstance();
    try {
      AutonomousSelector = new LoggedDashboardChooser<>(("Autonomous Selector"), AutoBuilder.buildAutoChooser()); 
    } catch (final Exception Ignored) {
      new Alert(("Autonomous Configuration Failed"), AlertType.ERROR);
      if(AutonomousSelector ==  (null)) {
        final var Selector = new SendableChooser<Command>();
        Selector.addOption(("Autonomous Configuration Failed"), new InstantCommand());
        AutonomousSelector = new LoggedDashboardChooser<>(("Autonomous Selector"), Selector); 
      }
    }
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized RobotContainer getInstance() {
      if (java.util.Objects.isNull(Instance)) {
          Instance = new RobotContainer();
      }
      return Instance;
  }
}