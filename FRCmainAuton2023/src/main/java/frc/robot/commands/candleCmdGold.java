package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.constant.Constable;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import frc.robot.subsystems.Other;
import frc.robot.subsystems.VerticalElevator;

public class candleCmdGold extends CommandBase {
    //define required subsystem
        private final Other m_Other;
 
        


    public candleCmdGold(Other subsystem) {

        //requires subsystem
        m_Other = subsystem;
        addRequirements(m_Other);

   
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //[subsystem].[command]
        m_Other.candleGold();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Other.candleOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

}
