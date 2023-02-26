package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.subsystems.Drive;



public class  driveTurnDegree extends CommandBase {

        private final Drive m_drive;
        public final double m_requestedTurn;

    public  driveTurnDegree(Drive subsystem, double requestedTurn) {



        m_drive = subsystem;
        addRequirements(m_drive);

        m_requestedTurn = requestedTurn;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.driveTurn(m_requestedTurn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_drive.turnDone();
        } 

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
