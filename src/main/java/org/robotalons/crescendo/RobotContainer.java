// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.subsystems.SubsystemManager;
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
  public static final List<LoggedDashboardChooser<Operator>> SubsystemOperators;
  public static final LoggedDashboardChooser<Command> Autonomous;
  // ------------------------------23---------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  
  private RobotContainer() {} static {
    SubsystemOperators = new ArrayList<>();
    SubsystemManager.getSubsystems().forEach((Subsystem) -> {
      final var Selector = new SendableChooser<Operator>();
      final var Default = Profiles.DEFAULT.get(Subsystem);
      Profiles.OPERATORS.forEach((Profile) -> Selector.addOption(Profile.getName(), Profile));
      Selector.setDefaultOption(Default.getName(), Default);
      Selector.onChange(Subsystem::configure);
      Subsystem.configure(Default);
      SubsystemOperators.add(new LoggedDashboardChooser<Operator>(Subsystem.getName() + " Pilot Selector", Selector));
    });
    Profiles.OPERATORS.forEach((Profile) -> SmartDashboard.putData(Profile.getName(), Profile));
    SubsystemManager.getInstance();
    Autonomous = new LoggedDashboardChooser<>(("Autonomous Selector"), AutoBuilder.buildAutoChooser());
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