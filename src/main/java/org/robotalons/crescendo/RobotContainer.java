// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder; //TODO: Reimplement
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.Constants.Pathplanner;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
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
    Autonomous = new LoggedDashboardChooser<>(("Autonomous Selector"), new SendableChooser<>()); //TODO: Reimplement Auto-builder
    Pathplanner.ROUTINES.forEach((Name, Routine) -> Autonomous.addOption(Name, Routine));

    //TODO: Remove Temporary Intake-Indexer-Shooter Code
    @SuppressWarnings("resource")
    final var INDEXER = new CANSparkMax((22), MotorType.kBrushless);
    INDEXER.setSmartCurrentLimit((20));
    INDEXER.setSecondaryCurrentLimit((30));
    INDEXER.setIdleMode(IdleMode.kBrake);
    INDEXER.setInverted((false));

    final var INTAKE = new CANSparkMax((21), MotorType.kBrushless);
    INTAKE.setSmartCurrentLimit((20));
    INTAKE.setSecondaryCurrentLimit((30));

    Profiles.OPERATORS.get(0).getKeybinding(Keybindings.INTAKE_TOGGLE).onTrue(new InstantCommand(() -> {
      INTAKE.set(-1d);
      INDEXER.set(1d);
    }));
    Profiles.OPERATORS.get(0).getKeybinding(Keybindings.INTAKE_TOGGLE).onFalse(new InstantCommand(() -> {
      INTAKE.set(0d);
      INDEXER.set(0d);
    }));
    Profiles.OPERATORS.get(0).getKeybinding(Keybindings.OUTTAKE_TOGGLE).onTrue(new InstantCommand(() -> {
      INTAKE.set(1d);
      INDEXER.set(-1d);
    }));
    Profiles.OPERATORS.get(0).getKeybinding(Keybindings.OUTTAKE_TOGGLE).onFalse(new InstantCommand(() -> {
      INTAKE.set(0d);
      INDEXER.set(0d);
    }));

    

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