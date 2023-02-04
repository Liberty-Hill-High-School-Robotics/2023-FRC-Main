// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class HorizontalRotate extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

private CANSparkMax cANSparkMAXHR;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    
    public RelativeEncoder encoderHR;

    public HorizontalRotate() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

 
 

cANSparkMAXHR = new CANSparkMax(10, MotorType.kBrushless);
 
 


 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    
    encoderHR  = cANSparkMAXHR.getEncoder();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    
    public void HRUp(){
        cANSparkMAXHR.set(0.5);
    }

    public void HRDown(){
        cANSparkMAXHR.set(0.5);
    }

    public void HRStop(){
        cANSparkMAXHR.stopMotor();
    }

   

    public boolean isHRRetracted(){
        return cANSparkMAXHR.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
    }


    
    public double HRGoTo(Constants.PlacementConstants.PlacementPosition position){
        Constants.PlacementConstants temp = new Constants.PlacementConstants();
        Double output = temp.getPlacementValues(position, Constants.PlacementConstants.SubSystem.ELBOW);
        return output;
    }


}

