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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 *
 */
public class ClawMotor extends SubsystemBase {

    private CANSparkMax clawMotor;
    private double power = 0.6;
    private double holdpower = .2;

    public ClawMotor() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        clawMotor = new CANSparkMax(11, MotorType.kBrushless);
        clawMotor.restoreFactoryDefaults();
        clawMotor.setInverted(false);
        clawMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void clawClose() {
        clawMotor.set(power);
    }

    public void clawOpen() {
        clawMotor.set(-power);
    }

    public void clawHold(){
        clawMotor.set(holdpower);
    }



}
