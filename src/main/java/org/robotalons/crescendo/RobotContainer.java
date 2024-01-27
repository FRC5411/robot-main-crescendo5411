// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.Constants.Pathplanner;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.Constants.Subsystems;
import org.robotalons.lib.utilities.PilotProfile;
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
  public static final List<LoggedDashboardChooser<PilotProfile>> PilotSelector;
  public static final LoggedDashboardChooser<Command> AutonomousSelector;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {} static {
    PilotSelector = new ArrayList<>();
    AutonomousSelector = new LoggedDashboardChooser<>(("Autonomous Selector"), AutoBuilder.buildAutoChooser());
    Pathplanner.ROUTINES.forEach((Name, Routine) -> AutonomousSelector.addOption(Name, Routine));
    Subsystems.SUBSYSTEMS.forEach((Subsystem) -> {
      final var Selector = new SendableChooser<PilotProfile>();
      final var Iterator = Profiles.PILOT_PROFILES.iterator();
      final var Initial = Iterator.next();
      Iterator.forEachRemaining((Profile) -> Selector.addOption(Profile.getName(), Profile));
      Selector.setDefaultOption(Initial.getName(), Initial);
      Selector.onChange(Subsystem::configure);
      Subsystem.configure(Initial);
      PilotSelector.add(new LoggedDashboardChooser<PilotProfile>(Subsystem.getName() + " Pilot Selector", Selector));
    });
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