// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.VerticalElevator;

import java.util.function.DoubleSupplier;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.VerticalElevator;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class VEGoTo extends CommandBase {
    
    private Constants.PlacementConstants.PlacementPosition m_position;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final VerticalElevator m_verticalElevator;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public VEGoTo(VerticalElevator subsystem, Constants.PlacementConstants.PlacementPosition position) {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_verticalElevator = subsystem;
        addRequirements(m_verticalElevator);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
       
        m_position = position;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_verticalElevator.VEGoTo(m_position);
        
    }
boolean goingUp = false;
boolean goingDown = false;
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_verticalElevator.encoderVE.getPosition() > m_verticalElevator.VEGoTo(m_position)){
            m_verticalElevator.VEDown();
            goingDown = true;
            
        }else if(m_verticalElevator.encoderVE.getPosition() < m_verticalElevator.VEGoTo(m_position)){
            m_verticalElevator.VEUp();
            goingUp = true;
        }

        if(goingUp == true){
            m_verticalElevator.VEUp();
        }else if(goingDown == true){
            m_verticalElevator.VEDown();;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_verticalElevator.VEStop();
    }

    // Returns true when the command should end.
    boolean end = false;
    @Override
    public boolean isFinished() {
        if(goingUp ==  true){
            if(m_verticalElevator.encoderVE.getPosition() >= m_verticalElevator.VEGoTo(m_position)){
                end = true;
             } else end  = false;
        
        }else if(goingDown == true){
            if(m_verticalElevator.encoderVE.getPosition() <= m_verticalElevator.VEGoTo(m_position)){
                end = true;
             } else end  = false;
        }
        return end;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
    
}