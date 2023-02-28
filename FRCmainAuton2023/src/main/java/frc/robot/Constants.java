// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
   /**
    * public static final class DriveConstants {
    *   public static final int kLeftMotor1Port = 0;
    *   public static final int kLeftMotor2Port = 1;
    *   public static final int kRightMotor1Port = 2;
    *   public static final int kRightMotor2Port = 3; 
    * }
    */ 
   
    public static final class PlacementConstants
    {
        public enum PlacementPosition 
        {
            FLOOR      (0),  //Position for picking placing up from the floor
            MIDDLE     (1), //Position for placing on the middle node
            PICKUP     (2),  //Position for picking up from the double station
            TOP        (3),   //Position for placing on the top node
            START      (4),
            //floor2
            FLOOR2UP   (5),
            FLOOR2DOWN (6)
            ;
            private final int placementPositionIndex;

            PlacementPosition(int placementPositionIndex){
                this.placementPositionIndex = placementPositionIndex;
            }
            
            public int getPlacementPostionIndex(){
                return this.placementPositionIndex;
            }

        }
        
        public enum SubSystem{
            VERTICAL    (0), // 
            HORIZONTAL  (1),
            ELBOW       (2),
            WRIST       (3);

            private final int placementSubSystemIndex;

            SubSystem (int placementSubSystemIndex){
                this.placementSubSystemIndex = placementSubSystemIndex;
            }

            public int getPlacementSubSystemIndex () {
                return this.placementSubSystemIndex;
            }


        }

        private double[][] placementValues = {
            {2,0.095,0.49,-40.28}, //floor
            {250,6.45,.74,7.3}, // middle
            {0,0,.767,16.1}, // pickup
            {8707,9.73,.62,-28.3}, // top
            {0,0,0.87,0}, // start
            //floor2 up & down
            {6900, 0,.467, 6.83}, //up
            {2800, 0, .467, 6.83} //down
        };

        public double getPlacementValues(PlacementPosition position, SubSystem subSystem) {
            double values = placementValues[position.getPlacementPostionIndex()][subSystem.getPlacementSubSystemIndex()];
            return values;
        }
        
    }
}

