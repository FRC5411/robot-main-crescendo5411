package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase
{
    private InfraredEmitter emitterOne;
    private InfraredEmitter emitterTwo;
    private InfraredReceiver receiverOne;
    private InfraredReceiver receiverTwo;
    public bool hasNote; //hasNote is THE bool value that the indexer keeps track of; everything else is secondary to making this accurate 100% of the time

    public Indexer()
    {
        emitterOne = new InfraredEmitter(0); //emitters each emit an IR light beam (if you can believe it)
        emitterTwo = new InfraredEmitter(1); //I think 2 IR beams will be more reliable than 1; no evidence except the obvious so I am going to research/test further

        receiverOne = new InfraredReceiver(2); //receivers return a 0 if the robot has a Note and a 1 if not
        receiverTwo = new InfraredReceiver(3); //Note will interrupt IR beam
    }
    
    @Override
    public void periodic()
    {
        hasNote = !(receiverOne.status() || receiverTwo.status()); //status() should return true for an unbroken beam
    }
}