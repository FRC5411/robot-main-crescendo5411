// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.indexer;

// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.Closeable;

// ------------------------------------------------------------[Indexer Subsystem]---------------------------------------------------------- //
/**
 *
 * <h1>IndexerSubsystem</h1>
 *
 * <p>Utility class which controls the indexing of notes from and to the intake and shooter.<p>
 * 
 * @see SubsystemBase
 * @see {@link org.robotalons.crescendo.RobotContainer RobotContainer} 
 */

public class IndexerSubsystem extends SubsystemBase implements Closeable
{
    // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
    //private static final InfraredReceiver ReceiverOne; //placeholder class
    //private static final InfraredReceiver ReceiverTwo; //placeholder class

    // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
    private static IndexerSubsystem Instance;
    private static Boolean ContainsNote; //the big one
    
    // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
    /* Indexer Subsystem Constructor */
    private IndexerSubsystem() {} static
    {
        ContainsNote = (false);
        //ReceiverOne = new InfraredReceiver(port: 0) //placeholder object declaration and port location
        //ReceiverTwo = new InfraredReceiver(port: 1) //placeholder object declaration and port location
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic()
    {
      Constants.Objects.ODOMETRY_LOCKER.lock();
      //ContainsNote = !(ReceiverOne.status || ReceiverTwo.status); //ContainsNote will be set to true if BOTH beams are broken; OR operator may become an AND operator depending on what is more/less reliable
      Constants.Objects.ODOMETRY_LOCKER.lock();
    }
    
    /* Closes this instance and all held resources immediately */
    public synchronized void close() { }
    
    // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
    
    // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
    
    // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
    /**
     * Provides a boolean representation of if this indexer is currently holding a not or not.
     * @return Boolean of if a note is being held
     */
    public Boolean getHoldingNote()
    {
        return ContainsNote;
    }
    
    /**
     * Retrieves the existing instance of this static utility class
     * @return Utility class's instance
     */
    public static synchronized IndexerSubsystem getInstance()
    {
        if (java.util.Objects.isNull(Instance))
            Instance = new IndexerSubsystem();
        return Instance;
    }
}