// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PlacementConstants.PlacementPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HorizontalElevator;
import frc.robot.subsystems.HorizontalRotate;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.VerticalElevator;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class PlaceTopAndBalance extends SequentialCommandGroup {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public PlaceTopAndBalance(
        Drive drive,
        Claw claw,
        GoToTop goToTop,
        GoToStart toStart,
        VerticalElevator verticalElevator,
        HorizontalElevator horizontalElevator,
        HorizontalRotate horizontalRotate,
        ClawRotate clawRoatate
    ){

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    addCommands(
        // Add Commands here:
        // Also add parallel commands using the
        //
        // addCommands(
        //      new command1(argsN, subsystem),
        //      parallel(
        //          new command2(argsN, subsystem),
        //          new command3(argsN, subsystem)
        //      )    
        //  );
        
        new PlaceConeTop(claw, goToTop, toStart, verticalElevator, horizontalElevator, horizontalRotate, clawRoatate),
        new WaitCommand(.5),
        new GoToMiddle(verticalElevator, horizontalElevator, horizontalRotate, clawRoatate),
        new driveBackPastCharged(drive),
        new WaitCommand(2),
        new driveForwardToCharged(drive),
        new WaitCommand(2),
        new driveBalance(drive)
        
        
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
