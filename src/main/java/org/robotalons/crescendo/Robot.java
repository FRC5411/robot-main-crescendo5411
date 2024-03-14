// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
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
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;
import org.robotalons.crescendo.Constants.Logging;
import org.robotalons.crescendo.Constants.Subsystems;
import org.robotalons.crescendo.subsystems.superstructure.SuperstructureSubsystem;
import org.robotalons.crescendo.subsystems.superstructure.SuperstructureSubsystem.SuperstructureState;
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
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Boolean CurrentAutonomousMessagePrinted;
  private static Double CurrentAutonomousStartTime;
  private static Command CurrentAutonomous;
  private static Robot Instance;
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private double lastChange;
  private boolean on = true;
  //private DigitalInput beamBreakSensor = new DigitalInput(1);
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
        new Alert(("GIT VCS CHANGES COMMITTED"), AlertType.INFO);
        break;
      case 1:
        Logger.recordMetadata(("Changes"), ("Uncommitted"));
        new Alert(("GIT VCS CHANGES NOT COMMITTED"), AlertType.INFO);
        break;
      default:
        Logger.recordMetadata(("Changes"), ("Unknown"));
        new Alert(("GIT VCS ERROR OR BUILD ISSUE"), AlertType.INFO);
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
    RobotContainer.getInstance();
    Shuffleboard.startRecording();
    DataLogManager.start();
    DriverStation.silenceJoystickConnectionWarning((true));
    PortForwarder.add((5800), ("photonvision.local"), (5800));

    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(26);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    
   
   m_led.setData(m_ledBuffer);
  }

  

  public void green(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);

   }
  }

  public void white(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 0);

   }
  }

  public void blinking(){
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp- lastChange > 0.1){
      on = !on;
      lastChange = timestamp;
    }
    if (on){
      green();
    } else {
      white();
    }
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
              ("*** Auto finished in %.2f secs ***%n"), Logger.getRealTimestamp() / (1e6) - CurrentAutonomousStartTime);
        } else {
          System.out.printf(
              ("*** Auto cancelled in %.2f secs ***%n"), Logger.getRealTimestamp() / (1e6) - CurrentAutonomousStartTime);
        }
        CurrentAutonomousMessagePrinted = true;
      }
    }
    Logger.recordOutput(("Match Time"), DriverStation.getMatchTime());

    if(!SuperstructureSubsystem.getIndexerSensor()){
      blinking();
    } else {
     green();
    }
    m_led.setData(m_ledBuffer);
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