// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PlacementConstants.PlacementPosition;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.CRGoTo;
import frc.robot.commands.ClawClose;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveOutCommunity;
import frc.robot.commands.GoToFloor;
import frc.robot.commands.GoToFloor2Down;
import frc.robot.commands.GoToFloor2Up;
import frc.robot.commands.GoToMiddle;
import frc.robot.commands.GoToPickUp;
import frc.robot.commands.GoToStart;
import frc.robot.commands.GoToTop;
import frc.robot.commands.HEButton;
import frc.robot.commands.HEExtend;
import frc.robot.commands.HEGoTo;
import frc.robot.commands.HERetract;
import frc.robot.commands.HEStop;
import frc.robot.commands.HRButton;
import frc.robot.commands.HRDown;
import frc.robot.commands.HRGoTo;
import frc.robot.commands.HRStop;
import frc.robot.commands.HRUp;
import frc.robot.commands.MaxSpeed;
import frc.robot.commands.PlaceConeTop;
import frc.robot.commands.PlaceTopAndBalance;
import frc.robot.commands.PlaceTopConeAndDriveOut;
import frc.robot.commands.RotateClawDown;
import frc.robot.commands.RotateClawStop;
import frc.robot.commands.RotateClawUp;
import frc.robot.commands.VEButton;
import frc.robot.commands.VEDown;
import frc.robot.commands.VEGoTo;
import frc.robot.commands.VEStop;
import frc.robot.commands.VEUp;
import frc.robot.commands.candleCmdGold;
import frc.robot.commands.candleCmdPurple;
import frc.robot.commands.candleCmdRainbow;
import frc.robot.commands.driveBalance;
import frc.robot.commands.slowMode;
import frc.robot.commands.turn;
import frc.robot.commands.zTurn;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HorizontalElevator;
import frc.robot.subsystems.HorizontalRotate;
import frc.robot.subsystems.Other;
import frc.robot.subsystems.VerticalElevator;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

//
//Shuffleboard Exports
//
 

//
//
//

  private static RobotContainer m_robotContainer = new RobotContainer();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    public final Drive m_drive = new Drive();
    public final Other m_other = new Other();
    public final Claw m_claw = new Claw();
    public final ClawRotate m_clawRotate = new ClawRotate();
    public final VerticalElevator m_VerticalElevator = new VerticalElevator();
    public final HorizontalElevator m_HorizontalElevator = new HorizontalElevator();
    public final HorizontalRotate m_HorizontalRotate = new HorizontalRotate();
    public final GoToTop m_GoToTop = new GoToTop(m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate);
    public final GoToStart m_GoToStart = new GoToStart(m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate);
    public final GoToFloor2Up m_GoToFloor2Up = new GoToFloor2Up(m_claw, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate);
   


// Joysticks
public final Joystick driverJoystick = new Joystick(0);
public final XboxController operatorController = new XboxController(1);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
   
    

    
   
    


  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {

    SmartDashboard.putData("MaxSpeed", new MaxSpeed( m_drive ));

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems


    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();
  

    // Configure default commands
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_drive.setDefaultCommand(new DriveArcade( m_drive ) );
    m_VerticalElevator.setDefaultCommand(new VEButton(m_VerticalElevator));
    m_HorizontalRotate.setDefaultCommand(new HRButton(m_HorizontalRotate));
    m_HorizontalElevator.setDefaultCommand(new HEButton(m_HorizontalElevator));


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
    m_chooser.addOption("PlaceConeTop", new PlaceConeTop(m_claw, m_GoToTop, m_GoToStart, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
      m_chooser.addOption("DriveOutCommunity", new DriveOutCommunity(m_drive));
      m_chooser.addOption("PlaceConeTopAndDriveOut", new PlaceTopConeAndDriveOut(m_drive, m_claw, m_GoToTop, m_GoToStart, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
   m_chooser.addOption("turn180", new turn(m_drive));

  m_chooser.addOption("PlaceTopAndBalance", new PlaceTopAndBalance(m_drive, m_claw, m_GoToTop, m_GoToStart, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    SmartDashboard.putData("Auto Mode", m_chooser);

  SmartDashboard.putData("RotateClawUp Command", new RotateClawUp(m_clawRotate));
    SmartDashboard.putData("RotateClawDown Command", new RotateClawDown(m_clawRotate));
    SmartDashboard.putData("RotateClawStop Command", new RotateClawStop(m_clawRotate));

    SmartDashboard.putData("ClawOpen Command", new ClawOpen(m_claw));
    SmartDashboard.putData("ClawClose Command", new ClawClose(m_claw));

    SmartDashboard.putData("VEUp Command", new VEUp(m_VerticalElevator));
    SmartDashboard.putData("VEDown Command", new VEDown(m_VerticalElevator));
    SmartDashboard.putData("VEStop Command", new VEStop(m_VerticalElevator));

    SmartDashboard.putData("HEExtend Command", new HEExtend(m_HorizontalElevator));
    SmartDashboard.putData("HERetract Command", new HERetract(m_HorizontalElevator));
    SmartDashboard.putData("HEStop Command", new HEStop(m_HorizontalElevator));

    SmartDashboard.putData("HRUp Command", new HRUp(m_HorizontalRotate));
    SmartDashboard.putData("HRDown Command", new HRDown(m_HorizontalRotate));
    SmartDashboard.putData("HRStop Command", new HRStop(m_HorizontalRotate));

    SmartDashboard.putData("GoToMiddle Command", new GoToMiddle(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
    SmartDashboard.putData("GoToFloor Command", new GoToFloor(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
    SmartDashboard.putData("GoToTop Command", new GoToTop(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
    SmartDashboard.putData("GoToStart Command", new GoToStart(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
    
    SmartDashboard.putData("driveBalance", new driveBalance(m_drive));

    SmartDashboard.putData("CRGoToFloor", new CRGoTo(m_clawRotate, PlacementPosition.FLOOR));
    SmartDashboard.putData("HRGoToFloor", new HRGoTo(m_HorizontalRotate, PlacementPosition.FLOOR));
    SmartDashboard.putData("HEGoToFloor", new HEGoTo(m_HorizontalElevator, PlacementPosition.FLOOR));
    SmartDashboard.putData("VEGoToFloor", new VEGoTo(m_VerticalElevator, PlacementPosition.FLOOR));

    SmartDashboard.putData("CRGoToMiddle", new CRGoTo(m_clawRotate, PlacementPosition.MIDDLE));
    SmartDashboard.putData("HRGoToMiddle", new HRGoTo(m_HorizontalRotate, PlacementPosition.MIDDLE));
    SmartDashboard.putData("HEGoToMiddle", new HEGoTo(m_HorizontalElevator, PlacementPosition.MIDDLE));
    SmartDashboard.putData("VEGoToMiddle", new VEGoTo(m_VerticalElevator, PlacementPosition.MIDDLE));

    SmartDashboard.putData("CRGoToTop", new CRGoTo(m_clawRotate, PlacementPosition.TOP));
    SmartDashboard.putData("HRGoToTop", new HRGoTo(m_HorizontalRotate, PlacementPosition.TOP));
    SmartDashboard.putData("HEGoToTop", new HEGoTo(m_HorizontalElevator, PlacementPosition.TOP));
    SmartDashboard.putData("VEGoToTop", new VEGoTo(m_VerticalElevator, PlacementPosition.TOP));

    SmartDashboard.putData("CRGoToPickup", new CRGoTo(m_clawRotate, PlacementPosition.PICKUP));
    SmartDashboard.putData("HRGoToPickup", new HRGoTo(m_HorizontalRotate, PlacementPosition.PICKUP));
    SmartDashboard.putData("HEGoToPickup", new HEGoTo(m_HorizontalElevator, PlacementPosition.PICKUP));
    SmartDashboard.putData("VEGoToPickup", new VEGoTo(m_VerticalElevator, PlacementPosition.PICKUP));

    SmartDashboard.putData("CRGoToStart", new CRGoTo(m_clawRotate, PlacementPosition.START));
    SmartDashboard.putData("HRGoToStart", new HRGoTo(m_HorizontalRotate, PlacementPosition.START));
    SmartDashboard.putData("HEGoToStart", new HEGoTo(m_HorizontalElevator, PlacementPosition.START));
    SmartDashboard.putData("VEGoToStart", new VEGoTo(m_VerticalElevator, PlacementPosition.START));

    SmartDashboard.putData("MaxSpeed", new MaxSpeed(m_drive));

    SmartDashboard.putData("LEDPurple", new candleCmdPurple(m_other));
    SmartDashboard.putData("LEDGold", new candleCmdGold(m_other));
    SmartDashboard.putData("LEDRainbow", new candleCmdRainbow(m_other));

    SmartDashboard.putData("DriveOutCommunity", new DriveOutCommunity(m_drive));
    SmartDashboard.putData("PlaceTopCone", new PlaceConeTop(m_claw, m_GoToTop, m_GoToStart, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));
    SmartDashboard.putData("PlaceConeTopAndDriveOut", new PlaceTopConeAndDriveOut(m_drive, m_claw, m_GoToTop, m_GoToStart, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));


  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }



  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
// Create some buttons


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
      //Button button 
 

      //left buttons, all go to certain posistions
      //lettered pad
    final Trigger buttonGoToMiddle = new JoystickButton(operatorController, XboxController.Button.kB.value);
    buttonGoToMiddle.whileTrue(new GoToMiddle(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate))
    .whileFalse(new GoToStart(m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));

    final Trigger buttonGoToTop = new JoystickButton(operatorController, XboxController.Button.kY.value);
    buttonGoToTop.whileTrue(new GoToTop(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate))
    .whileFalse(new GoToStart(m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));

    final Trigger buttonGoToFloor = new JoystickButton(operatorController, XboxController.Button.kA.value);
    buttonGoToFloor.whileTrue(new GoToFloor(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate))
    .whileFalse(new GoToStart(m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));

    final Trigger buttonGoToPickupPos = new JoystickButton(operatorController, XboxController.Button.kX.value);
    buttonGoToPickupPos.whileTrue(new GoToPickUp(m_VerticalElevator,m_HorizontalElevator, m_HorizontalRotate, m_clawRotate))
    .whileFalse(new GoToStart(m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));


    //
    //cone pickup thing
    final Trigger buttonFloor2Sequence = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    buttonFloor2Sequence.whileTrue(new GoToFloor2Up(m_claw, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate))
    .onFalse(new GoToFloor2Down(m_claw, m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate, m_GoToStart));

    //
    //axis stuff (weird!!!)
   



    //lights - driver joystick
    final Trigger buttonLEDPurple = new JoystickButton(driverJoystick, 3);
    buttonLEDPurple.toggleOnTrue(new candleCmdPurple(m_other));
    final Trigger buttonLEDGold = new JoystickButton(driverJoystick, 4); 
    buttonLEDGold.toggleOnTrue(new candleCmdGold(m_other));
   

    //
    //starting config and balance button
    //add balance here

    final Trigger buttonStartPos = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    buttonStartPos.onTrue(new GoToStart(m_VerticalElevator, m_HorizontalElevator, m_HorizontalRotate, m_clawRotate));



    //claw
    final Trigger buttonClawOpen = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    //buttonClawOpen.whileTrue(new ClawOpen(m_claw)).whileFalse( new ClawClose(m_claw));
   buttonClawOpen.onTrue(new ClawOpen(m_claw)).onFalse( new ClawClose(m_claw));

    

    final Trigger buttonBalance = new JoystickButton(driverJoystick, 5);
    buttonBalance.whileTrue(new driveBalance(m_drive));

    final Trigger buttonSlowMode = new JoystickButton(driverJoystick, 1);
    buttonSlowMode.whileTrue(new slowMode(m_drive));

    final Trigger zTurnAxis = new JoystickButton(driverJoystick, 2);
    zTurnAxis.whileTrue(new zTurn(m_drive));
    



     

    //driver joystick
 final Trigger buttonRotateClawUp = new JoystickButton(driverJoystick, 5);

  /* buttonRotateClawUp.onTrue(new RotateClawUp(m_clawRotate));
//
//   coded to x because trigger is not an option, change later
  final Trigger buttonRotateClawDown = new JoystickButton(driverJoystick, 6);
    buttonRotateClawDown.onTrue(new RotateClawDown(m_clawRotate));

    final Trigger buttonHRup = new JoystickButton(operatorController, XboxController.Button.kX.value);
    buttonHRup.whileTrue(new HRUp(m_HorizontalRotate));

    final Trigger buttonHRDown = new JoystickButton(operatorController, XboxController.Button.kA.value);
    buttonHRDown.whileTrue(new HRDown(m_HorizontalRotate));

    final Trigger buttonVEup = new JoystickButton(operatorController, XboxController.Button.kY.value);
    buttonVEup.whileTrue(new VEUp(m_VerticalElevator));

    final Trigger buttonVEDown = new JoystickButton(operatorController, XboxController.Button.kB.value);
    buttonVEDown.whileTrue(new VEDown(m_VerticalElevator));

    final Trigger testClawOpen = new JoystickButton(driverJoystick, 1);
    testClawOpen.onTrue(new ClawOpen(m_claw));

    final Trigger testClawClose = new JoystickButton(driverJoystick, 2);
    testClawClose.onTrue(new ClawClose(m_claw));

    final Trigger outHE = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    outHE.whileTrue(new HEExtend(m_HorizontalElevator));

    final Trigger inHE = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    inHE.whileTrue(new HERetract(m_HorizontalElevator));
*/
    
//
//
//
//  define button, add trigger, then attach command
/* 
    int rightBumper = XboxController.Button.kRightBumper.value;
    Trigger buttonClawOpen = new JoystickButton(operatorController, rightBumper);

    buttonClawOpen.whileTrue(new ClawOpen(m_claw));
    buttonClawOpen.whileFalse(new ClawClose(m_claw));
//
//
    int leftBumper = XboxController.Button.kLeftBumper.value;
    Trigger buttonRotateClawUp = new JoystickButton(operatorController, leftBumper);

    buttonRotateClawUp.whileTrue(new RotateClawUp(m_claw));
//
//   coded to x because trigger is not an option, change later
    int xButton = XboxController.Button.kX.value;
    Trigger buttonRotateClawDown = new JoystickButton(operatorController, xButton);

    buttonRotateClawDown.whileTrue(new RotateClawDown(m_claw));
//
*/
//



  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public Joystick getdriverJoystick() {
        return driverJoystick;
    }

  public XboxController getOperatorController() {
      return operatorController;
  }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
 
  

}

