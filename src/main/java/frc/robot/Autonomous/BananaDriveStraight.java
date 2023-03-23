package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.BananaDriveTrain;
import frc.robot.Robot.BananaConstants;
import frc.robot.Robot.Robot;

public class BananaDriveStraight {
    
    private static Timer clock = new Timer();
    



    public static void driveStraight(int distance)
    {

        //Timer.delay(BananaConstants.NAVX_RESET_WAIT_TIME);
        
        clock.reset();
        clock.start();
        
        switch (distance){

            case(0):    //straightForwardSide(0)
                Robot.driveTrain.tankDrive(-0.7, -0.7);
                
            break;

            case(1):    //straightForwardMid(1)
                Robot.driveTrain.tankDriveMid(-1, -1);
                
            break;

            case(2):    //tipToeForward(2)
                Robot.driveTrain.tankDriveLow(-1.3, -1.3);
                
            break;

            case(3):    //straightBackwardSide(3)
                Robot.driveTrain.tankDrive(0.7, 0.7);
                
            break;

            case(4):    //straightBackwardMid(4)
                Robot.driveTrain.tankDriveMid(1, 1);
                
            break;

            case(5):    //tipToeBackward(5)
                Robot.driveTrain.tankDriveLow(1.3, 1.3);
                
            break;

            case(6):    //rightTurn (6)
                Robot.driveTrain.tankDriveLow(-1.5, 1.5);

            break;

            case(7):    //leftTurn (7)
                Robot.driveTrain.tankDriveLow(1.5, -1.5);
            
            break;

            case(8):    //hyperRightTurn (8)
                Robot.driveTrain.tankDriveLow(-3.205, 3.205); //N x as fast as "rightTurn"

            break;

            case(9):    //hyperLeftTurn (9)
                Robot.driveTrain.tankDriveLow(3.205, -3.205); //N x as fast as "leftTurn"

            break;




                


            
        }


    }
}
