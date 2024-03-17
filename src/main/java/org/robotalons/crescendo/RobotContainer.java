// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;

import org.robotalons.crescendo.Constants.Odometry;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.Constants.Subsystems;
import org.robotalons.crescendo.subsystems.SubsystemManager;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.lib.utilities.Alert;
import org.robotalons.lib.utilities.Alert.AlertType;
import org.robotalons.lib.utilities.LoggedDashboardChooser;
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
  public static final List<LoggedDashboardChooser<Operator<Keybindings, Preferences>>> Operators;
  public static LoggedDashboardChooser<Command> Autonomous;
  public static LoggedDashboardChooser<Pose2d> Location;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Robot Container Constructor
   */
  private RobotContainer() {} static {
    Operators = new ArrayList<>();
    SubsystemManager.getInstance();
    SubsystemManager.getSubsystems().forEach((Subsystem) -> {
      final var Selector = new SendableChooser<Operator<Keybindings, Preferences>>();
      Selector.onChange(Subsystem::configureOperator);
      final var Default = Profiles.DEFAULT.get(Subsystem);
      Profiles.OPERATORS.forEach((Profile) -> Selector.addOption(Profile.getName(), Profile));
      Selector.setDefaultOption(Default.getName(), Default);
      Subsystem.configureOperator(Default);
      Operators.add(new LoggedDashboardChooser<>(Subsystem.getName() + " Operator Selector", Selector));
    });
    final var Alliance = DriverStation.getAlliance();
    final var Selector = new SendableChooser<Pose2d>();
    Selector.onChange(DrivebaseSubsystem::set);
    for(Integer Index = (1); Index < Odometry.ALLIANCE_VERTICAL_LOCATIONS.size() + (1); Index++) {
      final var Location = Odometry.ALLIANCE_VERTICAL_LOCATIONS.get(Index - (1));
      if(Index == (RobotBase.isReal()? DriverStation.getLocation().getAsInt(): (Subsystems.DEFAULT_ALLIANCE))) {
        Selector.setDefaultOption(String.format(("%s Alliance %d"), Alliance.map(Enum::name).orElseGet(() -> (!DrivebaseSubsystem.getFlipped() ? "Blue" : "Red")), Index), new Pose2d(Odometry.ALLIANCE_HORIZONTAL_LOCATIONS, Location, DrivebaseSubsystem.getRotation()));
      } else {
        Selector.addOption(String.format(("%s Alliance %d"), Alliance.map(Enum::name).orElseGet(() -> (!DrivebaseSubsystem.getFlipped() ? "Blue" : "Red")), Index), new Pose2d(Odometry.ALLIANCE_HORIZONTAL_LOCATIONS, Location, DrivebaseSubsystem.getRotation()));
      }
    }
    Selector.addOption(("Debug Alliance"), new Pose2d());
    DrivebaseSubsystem.set(Selector.getSelected() == (null)? new Pose2d(): Selector.getSelected());
    Location = new LoggedDashboardChooser<>(("Location Selector"), Selector);
    try {
      final var AutoSelector = AutoBuilder.buildAutoChooser();
      AutoSelector.onChange(SubsystemManager::set);
      Autonomous = new LoggedDashboardChooser<>(("Autonomous Selector"), AutoSelector); 
    } catch (final Exception Ignored) {
      new Alert(("Autonomous Unavailable"), AlertType.ERROR);
      if(Autonomous ==  (null)) {
        final var AutoSelector = new SendableChooser<Command>();
        AutoSelector.addOption(("Autonomous Configuration Failed"), new InstantCommand());
        Autonomous = new LoggedDashboardChooser<>(("Autonomous Selector"), AutoSelector); 
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