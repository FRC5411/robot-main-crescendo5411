// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.indexer;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ------------------------------------------------------------[Indexer Subsystem]----------------------------------------------------------//
/**
 *
 *
 * <h1>IndexerSubsystem</h1>
 *
 * <p>Utility class which controls the indexing of notes from and to the intake and shooter.<p>
 * 
 * @see SubsystemBase
 * @see {@link org.robotalons.crescendo.RobotContainer RobotContainer} 
 */
public class IndexerSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  //private static final InfraredEmitter EmitterLeft;    //<---- This class does not exist, is it a placeholder? @SpiderFace
  //private static final InfraredEmitter EmitterRight; 
  //private static final InfraredReceiver ReceiverLeft;  //<---- This class does not exist, is it a placeholder? @SpiderFace
  //private static final InfraredReceiver ReceiverRight;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static IndexerSubsystem Instance;
  private static Boolean ContainsNote;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private IndexerSubsystem() {} static {
    ContainsNote = (false);
  }
  
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic()
  {
    //ContainsNote = !(ReceiverLeft.status() || ReceiverRight.status()); //status() should return true for an unbroken beam
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides a boolean representation of if this indexer is currently holding a not or not.
   * @return Boolean of if a note is being held
   */
  public Boolean getHoldingNote() {
    return ContainsNote;
  }

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized IndexerSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance)) {
        Instance = new IndexerSubsystem();
      }
      return Instance;
  }
}