// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.robotalons.crescendo.Constants.Logging;
import org.robotalons.crescendo.Constants.Subsystems;
import org.robotalons.crescendo.subsystems.SubsystemManager;
import org.robotalons.lib.motion.utilities.CTREOdometryThread;
import org.robotalons.lib.motion.utilities.REVOdometryThread;

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

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Boolean CurrentAutonomousMessagePrinted;
  private static Double CurrentAutonomousStartTime;
  private static Command CurrentAutonomous;
  private static Robot Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private Robot() {} static {
    CurrentAutonomous = (null);
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
    Logger.recordMetadata(("ProjectName"), BuildMetadata.MAVEN_NAME);
    Logger.recordMetadata(("BuildDate"), BuildMetadata.BUILD_DATE);
    Logger.recordMetadata(("GitSHA"), BuildMetadata.GIT_SHA);
    Logger.recordMetadata(("GitDate"), BuildMetadata.GIT_DATE);
    Logger.recordMetadata(("GitBranch"), BuildMetadata.GIT_BRANCH);
    switch (BuildMetadata.DIRTY) {
      case 0:
        Logger.recordMetadata(("Changes"), ("Committed"));
        break;
      case 1:
        Logger.recordMetadata(("Changes"), ("Uncommitted"));
        break;
      default:
        Logger.recordMetadata(("Changes"), ("Unknown"));
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
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
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
    RobotContainer.getInstance();
    Shuffleboard.startRecording();

  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority((true), (99));
    CommandScheduler.getInstance().run();
    SmartDashboard.updateValues();
    if (CurrentAutonomous != (null)) {
      if (!CurrentAutonomous.isScheduled() && !CurrentAutonomousMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              ("*** Auto finished in %.2f secs ***%n"), Timer.getFPGATimestamp() - CurrentAutonomousStartTime);
        } else {
          System.out.printf(
              ("*** Auto cancelled in %.2f secs ***%n"), Timer.getFPGATimestamp() - CurrentAutonomousStartTime);
        }
        CurrentAutonomousMessagePrinted = true;
      }
    }
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
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
      super.disabledExit();
  }

  // ------------------------------------------------------------[Autonomous]---------------------------------------------------------------//
  @Override
  public void autonomousInit() {
    SubsystemManager.ensure();
    CurrentAutonomousMessagePrinted = (false);
    CurrentAutonomousStartTime = Timer.getFPGATimestamp();
    CurrentAutonomous = RobotContainer.AutonomousSelector.get();
    if(!java.util.Objects.isNull(CurrentAutonomous)) {
      CurrentAutonomous.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if(!java.util.Objects.isNull(CurrentAutonomous)) {
      CurrentAutonomous.cancel();
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