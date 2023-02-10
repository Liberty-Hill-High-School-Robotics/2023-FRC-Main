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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 *
 */
public class Claw extends SubsystemBase {

    private DoubleSolenoid doubleSolenoidClaw;
    private Compressor compressor;
    

    /**
    *
    */
    public Claw() {
        
        doubleSolenoidClaw = new DoubleSolenoid(18, PneumaticsModuleType.CTREPCM, 0, 4);
        addChild("Claw Open Close", doubleSolenoidClaw);
        
        compressor = new Compressor(18, PneumaticsModuleType.CTREPCM);
        addChild("Compressor", compressor);

        compressor.enableDigital();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("CompressorPSI", compressor.getPressureSwitchValue());
       // SmartDashboard.putBoolean("Claw Closed", doubleSolenoidClaw.isFwdSolenoidDisabled());
       // SmartDashboard.putBoolean("Claw Open", doubleSolenoidClaw.isRevSolenoidDisabled());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void clawClose() {
        doubleSolenoidClaw.set(Value.kForward);
    }

    public void clawOpen() {
        doubleSolenoidClaw.set(Value.kReverse);
    }
 /* 
    public Boolean isClawOpen(){
        return doubleSolenoidClaw.isRevSolenoidDisabled();
    }
    
    public Boolean isClawClosed(){
        return doubleSolenoidClaw.isFwdSolenoidDisabled();
    }
    */

}
