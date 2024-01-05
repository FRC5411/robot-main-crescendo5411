// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//S
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.TimedRobot;

// ---------------------------------------------------------------[Robot Class]-------------------------------------------------------------//
/**
 * 
 * 
 * <h1> Robot </h1> 
 * 
 * <<p>Utility class which defines all modes of robot's event-cycle throughout it's lifetime.</p>>
 * 
 * @see RobotContainer
 */
public final class Robot extends TimedRobot  {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  public static Robot INSTANCE = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private Robot() {

  }

  // ---------------------------------------------------------------[Robot]-----------------------------------------------------------------//
  @Override
  public void robotInit() {
    Shuffleboard.startRecording();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();    
    SmartDashboard.updateValues();    
  }

  // ------------------------------------------------------------[Simulation]---------------------------------------------------------------//
  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }

  // -------------------------------------------------------------[Disabled]----------------------------------------------------------------//
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {

  }


  @Override
  public void disabledExit() {
    super.disabledExit();
  }
  // ------------------------------------------------------------[Autonomous]---------------------------------------------------------------//  
  @Override
  public void autonomousInit() {
  }


  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {

  }

  // -----------------------------------------------------------[Teleoperated]--------------------------------------------------------------//
  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {

  }     

  // ----------------------------------------------------------------[Test]------------------------------------------------------------------//
  @Override
  public void testPeriodic() {

  } 

  @Override
  public void testInit() {
    super.testInit();
  }

  @Override
  public void testExit() {
    super.testExit();
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public static synchronized Robot getInstance() {
    if(java.util.Objects.isNull(INSTANCE)) {
      INSTANCE = new Robot();
    }
    return INSTANCE;
  }
}