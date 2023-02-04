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


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 *
 */
public class HorizontalRotate extends SubsystemBase {

private CANSparkMax horizontalRotatorMotor;

    private double rotatePower = 0.5;

    private SparkMaxLimitSwitch forwardLimit;
    private SparkMaxLimitSwitch reverseLimit;

    /**
    *
    */
    
    public RelativeEncoder encoderHR;
    private double targetPosition;
    private double error = 50;

    public HorizontalRotate() {

horizontalRotatorMotor = new CANSparkMax(10, MotorType.kBrushless);
horizontalRotatorMotor.restoreFactoryDefaults();
horizontalRotatorMotor.setInverted(false);

forwardLimit = horizontalRotatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
reverseLimit = horizontalRotatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

forwardLimit.enableLimitSwitch(true);
reverseLimit.enableLimitSwitch(true);

    encoderHR  = horizontalRotatorMotor.getEncoder();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Horizontal Rotator Position", encoderHR.getPosition());
        SmartDashboard.putNumber("Horizontal Rotator Velocity", encoderHR.getVelocity());

        SmartDashboard.putBoolean("Forward Limit Switch", forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimit.isPressed());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    
    public void HRUp(){
        horizontalRotatorMotor.set(rotatePower);
    }

    public void HRDown(){
        horizontalRotatorMotor.set(-rotatePower);
    }

    public void HRStop(){
        horizontalRotatorMotor.stopMotor();
    }

   

    public boolean isHRRetracted(){
        return horizontalRotatorMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
    }


    
    public void HRGoTo(Constants.PlacementConstants.PlacementPosition position){
        Constants.PlacementConstants temp = new Constants.PlacementConstants();
        targetPosition = temp.getPlacementValues(position, Constants.PlacementConstants.SubSystem.HORIZONTAL);

        if(encoderHR.getPosition() > targetPosition){
            HRDown();
            
            
        }else if(encoderHR.getPosition() < targetPosition){
           
            HRUp();
        }

       
            

       
    }

   public boolean isHRAtPosition(){
        
        return Math.abs(encoderHR.getPosition() - targetPosition) <= error;
   }

}
