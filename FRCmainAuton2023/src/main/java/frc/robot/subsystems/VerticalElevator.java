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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

/**
 *
 */
public class VerticalElevator extends SubsystemBase {

    private CANSparkMax VEleadMotor;
    private CANSparkMax VEfollowMotor;

    public RelativeEncoder encoderVE;
    private double error = 200;
    private double targetPosition;
  // public DutyCycleEncoder throughBorVE;
    private Double ratePowerUp = 0.7;
    private Double ratePowerDown = -0.7;
    public Encoder relativeEncoderVE;
    private int elevatorError;

    private SparkMaxLimitSwitch forwardLimit;
    private SparkMaxLimitSwitch reverseLimit;

    public VerticalElevator() {

        VEleadMotor = new CANSparkMax(5, MotorType.kBrushless);
        VEleadMotor.restoreFactoryDefaults();
       
        VEleadMotor.setInverted(true);
        encoderVE = VEleadMotor.getEncoder();
        
        VEleadMotor.setIdleMode(IdleMode.kBrake);

        VEfollowMotor = new CANSparkMax(6, MotorType.kBrushless);
        VEfollowMotor.restoreFactoryDefaults();
        VEfollowMotor.follow(VEleadMotor, false);
        
        VEfollowMotor.setIdleMode(IdleMode.kBrake);

       // throughBorVE = new DutyCycleEncoder(1);

        relativeEncoderVE = new Encoder(1, 2);
        relativeEncoderVE.setReverseDirection(true);

        forwardLimit = VEleadMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit = VEleadMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        forwardLimit.enableLimitSwitch(true);
        reverseLimit.enableLimitSwitch(true);

        VEleadMotor.setSmartCurrentLimit(28);
        VEfollowMotor.setSmartCurrentLimit(28);

        

        

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(isVEAtBottom()){
            elevatorError = relativeEncoderVE.get();
        }

        SmartDashboard.putNumber("VEEncoder", encoderVE.getPosition());
        //SmartDashboard.putNumber("ThroughBorVE", throughBorVE.getAbsolutePosition());
        SmartDashboard.putNumber("RelativeEncoderVE", relativeEncoderVE.get());
        SmartDashboard.putNumber("VETarget Position", targetPosition);
        SmartDashboard.putNumber("ElevatorError", elevatorError);
        SmartDashboard.putBoolean("VEBottom LimitSwitch", isVEAtBottom());
        SmartDashboard.putBoolean("VETop LimitSwitch", isVEAtTop());

        SmartDashboard.putNumber("VE5motor", VEleadMotor.getMotorTemperature());
        SmartDashboard.putNumber("VE6motor", VEfollowMotor.getMotorTemperature());
       
        


    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void VEUp() {

        VEleadMotor.set(ratePowerUp);
    }

    public void VEDown() {
        VEleadMotor.set(ratePowerDown);
    }

    public void VEStop() {
        VEleadMotor.stopMotor();
    }

    public boolean isVEAtTop() {
        return VEleadMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();

    }

    public boolean isVEAtBottom() {
        return VEleadMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public void VEGoTo(Constants.PlacementConstants.PlacementPosition position) {
        Constants.PlacementConstants temp = new Constants.PlacementConstants();
        targetPosition = (temp.getPlacementValues(position, Constants.PlacementConstants.SubSystem.VERTICAL)
                + elevatorError);

        if (relativeEncoderVE.get() < targetPosition) {
            VEUp();

        } else if (relativeEncoderVE.get() > targetPosition) {

            VEDown();
        }

    }

    public boolean isVEAtPosition() {

        return Math.abs(relativeEncoderVE.get() - targetPosition) <= error;
    }

    public void zeroElevator() {
        if (isVEAtBottom() != true) {
            VEDown();
        }else 
        elevatorError = relativeEncoderVE.get();
    }

    public void VEHoldPosition(){
        VEleadMotor.set(.025);
    }

}
