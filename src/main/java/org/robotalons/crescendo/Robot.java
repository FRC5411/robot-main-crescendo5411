// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.robotalons.crescendo.Constants.Logging;
import org.robotalons.crescendo.Constants.Ports;
import org.robotalons.crescendo.Constants.Subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.stream.Stream;

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
    private static final RepeatCommand COMMAND_LOGGER = new RepeatCommand(new InstantCommand(() -> {
      if(Logging.LOGGING_ENABLED) {
        Threads.setCurrentThreadPriority((true), (99));
        List<String> ClientNames, ClientAddresses;
        ClientNames = new ArrayList<>(); ClientAddresses = new ArrayList<>();
        Stream.of(NetworkTableInstance.getDefault().getConnections()).forEach((Connection) -> {
          ClientNames.add(Connection.remote_id);
          ClientAddresses.add(Connection.remote_ip);
        });
        Logger.recordOutput(("NTClient/Names"), ClientNames.toArray(String[]::new));
        Logger.recordOutput(("NTClient/Addresses"), ClientAddresses.toArray(String[]::new));
        Threads.setCurrentThreadPriority((true), (20));      
      }
    }));
    // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
    private static Robot INSTANCE = (null);

    // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
    private Robot() {} static {
      
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
      if (Subsystems.IS_REAL_ROBOT) {
        if(Logging.LOGGING_ENABLED) {
          Logger.addDataReceiver(new WPILOGWriter(("/media/sda1/")));
        }
        Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance((Ports.POWER_DISTRIBUTION_HUB), ModuleType.kRev);
      } else {
        if(Logging.REPLAY_FROM_LOG) {
          setUseTiming(Logging.LOGGING_TURBO_MODE);
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          if(Logging.LOGGING_ENABLED) {
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, ("_sim"))));
          }
          } else {
            if(Logging.LOGGING_ENABLED) {
              Logger.addDataReceiver(new WPILOGWriter(Logging.LOGGING_DEPOSIT_FOLDER));
            }
            Logger.addDataReceiver(new NT4Publisher());        
          }
      }
      HashMap<String,Integer> CommandInstanceCount = new HashMap<>();
      BiConsumer<Command, Boolean> CommandFunctionLogger = 
        (Command Operation, Boolean Active) -> new Thread(() -> {
          String OperationName = Operation.getName();
          int Count = CommandInstanceCount.getOrDefault(OperationName, (0)) + ((Active)? (1): (-1));
          CommandInstanceCount.put(OperationName,Count);
          Logger.recordOutput("UniqueOperations/" + OperationName + "_" + Integer.toHexString(Operation.hashCode()), Active);
          Logger.recordOutput("Operations/" + OperationName, Count > (0));
        });
      CommandScheduler.getInstance().onCommandInitialize(
        (Command Command) -> CommandFunctionLogger.accept(Command, (true)));
      CommandScheduler.getInstance().onCommandInterrupt(
        (Command Command) -> CommandFunctionLogger.accept(Command, (false)));
      CommandScheduler.getInstance().onCommandFinish(
        (Command Command) -> CommandFunctionLogger.accept(Command, (false)));    
      Logger.start();
      for (int ForwardingPort = (5800); ForwardingPort <= (5805); ForwardingPort++) {
        PortForwarder.add(ForwardingPort, ("limelight.local"), ForwardingPort);
      }
      RobotContainer.getInstance();
      COMMAND_LOGGER.schedule();    
      Shuffleboard.startRecording();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.updateValues();
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
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

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
        if (java.util.Objects.isNull(INSTANCE)) {
            INSTANCE = new Robot();
        }
        return INSTANCE;
    }
}
