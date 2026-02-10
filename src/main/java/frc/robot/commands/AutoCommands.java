package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands {
    private static Drive drive = Drive.getInstance()
    private static Shooter shooter = Shooter.getInstance()
    private static Indexer indexer = Indexer.getInstance()
}

private AutoCommands(){

}

public static Command shoot(){
    return Commands.sequence(
        indexer
    )
}
