// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;
import org.robotalons.crescendo.Constants.Logging;
import org.robotalons.crescendo.Constants.Odometry;
import org.robotalons.crescendo.Constants.Subsystems;
import org.robotalons.lib.motion.utilities.CTREOdometryThread;
import org.robotalons.lib.motion.utilities.REVOdometryThread;
import org.robotalons.lib.utilities.Alert;
import org.robotalons.lib.utilities.Alert.AlertType;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import javax.management.InstanceNotFoundException;
// -----------------------------------------------------------------[Robot]----------------------------------------------------------------//
/**
 *
 *
 * <h1>Robot</h1>
 *
 * <p>Utility class which defines all modes of robot's event-cycle throughout it's lifetime.
 *
 * @see RobotContainer
 */
public final class Robot extends LoggedRobot {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final Alert BATTERY_VOLTAGE_ALERT = new Alert(("Battery Voltage Low"), AlertType.WARNING);
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Command AutonomousCurrent;  
  private static Boolean MessagePrinted;
  private static Double StartTimestamp;
  private static Robot Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private Robot() {} static {
    AutonomousCurrent = (null);
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * Describes the type of robot being initialized
   */
  public enum RobotType {
    SIMULATION,
    CONCRETE,
    REPLAY
  }
  // ---------------------------------------------------------------[Robot]-----------------------------------------------------------------//
  @Override
  @SuppressWarnings("ExtractMethodRecommender")
  public void robotInit() {
    PhotonCamera.setVersionCheckEnabled((false));
    Logger.recordMetadata(("ProjectName"), BuildMetadata.MAVEN_NAME);
    Logger.recordMetadata(("BuildDate"), BuildMetadata.BUILD_DATE);
    Logger.recordMetadata(("GitSHA"), BuildMetadata.GIT_SHA);
    Logger.recordMetadata(("GitDate"), BuildMetadata.GIT_DATE);
    Logger.recordMetadata(("GitBranch"), BuildMetadata.GIT_BRANCH);
    Logger.recordMetadata(("PhotonBuildDate"), PhotonVersion.buildDate);
    Logger.recordMetadata(("PhotonVersion"), PhotonVersion.versionString);
    Logger.recordMetadata(("PhotonIsRelease"), Boolean.toString(PhotonVersion.isRelease));
    switch (BuildMetadata.DIRTY) {
      case 0:
        Logger.recordMetadata(("Changes"), ("Committed"));
        break;
      case 1:
        Logger.recordMetadata(("Changes"), ("Uncommitted"));
        new Alert(("GIT VCS Changes Not Committed"), AlertType.INFO);
        break;
      default:
        Logger.recordMetadata(("Changes"), ("Unknown"));
        new Alert(("GIT VCS Changes Unknown"), AlertType.INFO);
        break;
    }
    switch (Constants.Subsystems.TYPE) {
      case CONCRETE:
        if (Logging.LOGGING_ENABLED) {
          String Folder = Constants.Logging.LOGGING_DEPOSIT.get(RobotType.CONCRETE);
          Logger.addDataReceiver(new WPILOGWriter(Folder));
        }  
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIMULATION:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming((false));
        String Path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(Path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(Path, ("_sim"))));
        break;
    }
    Logger.start();
    Map<String, Integer> CommandMap = new HashMap<>();
    BiConsumer<Command, Boolean> LoggerFunction =
        (Command command, Boolean active) -> {
          String CommandName = command.getName();
          int InstanceCount = CommandMap.getOrDefault(CommandName, (0)) + (active ? 1 : -1);
          CommandMap.put(CommandName, InstanceCount);
          Logger.recordOutput(
              "CommandsUnique/" + CommandName + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + CommandName, InstanceCount > (0));
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              LoggerFunction.accept(command, (true));
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              LoggerFunction.accept(command, (false));
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              LoggerFunction.accept(command, (false));
            });
    if (Subsystems.TYPE == RobotType.SIMULATION) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    } 
    try {
      CTREOdometryThread.getInstance();
      REVOdometryThread.getInstance();
    } catch (final InstanceNotFoundException Exception) {}
    Logger.registerURCL(URCL.startExternal());
    RobotController.setBrownoutVoltage((9d));
    RobotContainer.getInstance();
    Shuffleboard.startRecording();
    DataLogManager.start();
    DriverStation.silenceJoystickConnectionWarning((true));
    PortForwarder.add((5800), ("photonvision.local"), (5800));
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority((true), (99));
    BATTERY_VOLTAGE_ALERT.set(RobotController.getBatteryVoltage() < (11.5d));
    CommandScheduler.getInstance().run();
    SmartDashboard.updateValues();
    if (AutonomousCurrent != (null)) {
      if (!AutonomousCurrent.isScheduled() && !MessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              ("*** Auto finished in %.2f secs ***%n"), Logger.getRealTimestamp() / (1e6) - StartTimestamp);
        } else {
          System.out.printf(
              ("*** Auto cancelled in %.2f secs ***%n"), Logger.getRealTimestamp() / (1e6) - StartTimestamp);
        }
        MessagePrinted = (true);
      }
    }
    Logger.recordOutput(("Match Time"), DriverStation.getMatchTime());
    Threads.setCurrentThreadPriority((true), (10));
  }

  // ------------------------------------------------------------[Simulation]---------------------------------------------------------------//
  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  // -------------------------------------------------------------[Disabled]----------------------------------------------------------------//
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    Odometry.CTRE_ODOMETRY_THREAD.setEnabled((false));
    Odometry.REV_ODOMETRY_THREAD.setEnabled((false));
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    Odometry.CTRE_ODOMETRY_THREAD.setEnabled((true));
    Odometry.REV_ODOMETRY_THREAD.setEnabled((true));
  }

  // ------------------------------------------------------------[Autonomous]---------------------------------------------------------------//
  @Override
  public void autonomousInit() {
    MessagePrinted = (false);
    StartTimestamp = Timer.getFPGATimestamp();
    AutonomousCurrent = RobotContainer.AutonomousSelector.get();
    if(!java.util.Objects.isNull(AutonomousCurrent)) {
      AutonomousCurrent.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if(!java.util.Objects.isNull(AutonomousCurrent)) {
      AutonomousCurrent.cancel();
    }
  }

  // -----------------------------------------------------------[Teleoperated]--------------------------------------------------------------//
  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  // ----------------------------------------------------------------[Test]------------------------------------------------------------------//
  @Override
  public void testPeriodic() {}

  @Override
  public void testInit() {
      super.testInit();
  }

  @Override
  public void testExit() {
      super.testExit();
  }

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized Robot getInstance() {
      if (java.util.Objects.isNull(Instance)) {
          Instance = new Robot();
      }
      return Instance;
  }
}