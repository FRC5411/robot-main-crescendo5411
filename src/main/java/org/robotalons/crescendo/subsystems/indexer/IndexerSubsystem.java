// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.indexer;

// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import org.robotalons.crescendo.subsystems.indexer.Constants.Measurements;
import org.robotalons.crescendo.subsystems.indexer.Constants.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.io.Closeable;
import java.util.List;

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

public class IndexerSubsystem extends SubsystemBase implements Closeable {
    // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
    private static CANSparkMax m_IntakeIndexer;     //pulls Note from intake
    private static CANSparkMax m_ShooterIndexer;    //pushes Note to shooter
    
    // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
    private static IndexerSubsystem Instance;
    private static Boolean HasNote;
    
    private static InfraredReceiver IntakeIndexer;  //intake-side IR sensor (placeholder class)
    private static InfraredReceiver ShooterIndexer; //shooter-side IR sensor (placeholder class)
    
    // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
    /**
     * Indexer Subsystem Constructor
     */
    private IndexerSubsystem() {} static {
        Instance = new IndexerSubsystem();
        HasNote = (false);

        ReceiverOne = new InfraredReceiver(port: 0) //placeholder declaration and port location
        ReceiverTwo = new InfraredReceiver(port: 1) //placeholder declaration and port location

        m_IntakeIndexer = new CANSparkMax(Constants.INTAKE_INDEXER_MOTOR_ID, MotorType.kBrushless);
        m_ShooterIndexer = new CANSparkMax(Constants.Ports.SHOOTER_INDEXER_MOTOR_ID, MotorType.kBrushless);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
        Constants.Objects.ODOMETRY_LOCKER.lock();
        HasNote = !(ReceiverOne.status || ReceiverTwo.status); //set to true if BOTH beams are broken (OR may become an AND)
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
    public Boolean getHoldingNote() {
        return HasNote;
    }
    
    /**
     * Retrieves the existing instance of this static utility class
     * @return Utility class's instance
     */
    public static synchronized IndexerSubsystem getInstance() {
        if (java.util.Objects.isNull(Instance))
            Instance = new IndexerSubsystem();
        return Instance;
    }
}