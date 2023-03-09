package frc.robot.Autonomous;


import frc.robot.Mechanisms.BananaClaw;
import frc.robot.Robot.BananaPreSets;
import frc.robot.Robot.Robot;

public class BananaAutonPaths {





    





    private final static double leftHalfTurn = 90.0; //"left turn" means the front right corner whips around CCW
    private final static double leftFullTurn = 180;
    private final static double rightHalfTurn = -90.0; //"right turn" means the front left corner whips around CW
    private final static double rightFullTurn = -180;

    

    private final static int straightForwardSide = 0;
    private final static int straightForwardMid = 1;
    private final static int tipToeForward = 2; //Used for inching onto the charging station and for approaching Scoring

    private final static int straightBackwardSide = 3;
    private final static int straightBackwardMid = 4;
    private final static int tipToeBackward = 5; //Using for inching into the charging station and for approaching Scoring

    

    /*IMPORTANT: FIGURE OUT HOW TO PRELOAD OBJECTS INTO THE CLAW*/
    /*Also Important: Consider using a timer to seperate each command by 0.2 seconds so that error doesn't stack */


    //TEST HOW LONG IT TAKES TO CHANGE BETWEEN CERTAIN ANGLES
    //TEST HOW LONG IT TAKES TO OPEN AND CLOSE THE CLAW
    //TEST HOW LONG IT TAKES TO 
    

   






    public static void rightScoreMob() //turns left, away from the wall on the right side of the robot
    {   

        
        
        


        //comment out everything and uncomment each line after testing each time

        if(Robot.autonClock.get()>0.05 && Robot.autonClock.get()<1.1)
        BananaPreSets.cargoPickUp();
        if(Robot.autonClock.get()>=0.1 && Robot.autonClock.get()<0.15)
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second, operation time of 1 second
        if(Robot.autonClock.get()>=1.5 && Robot.autonClock.get()<2.1)
        BananaClaw.closeClaw(1); //in order to do the flippy, we need to grab the cone from the top side to allow the center of mass distribution to be on the bottom side, rotating the cone to face bottom down
        if (Robot.autonClock.get()>=2.1 && Robot.autonClock.get()<2.15)
        BananaClaw.closeClaw(0);
        if(Robot.autonClock.get()>=2.15 && Robot.autonClock.get()<3.15)
        BananaPreSets.lvl3RocketBall();
        if(Robot.autonClock.get()>=3.15 && Robot.autonClock.get()<3.30)
        BananaClaw.openClaw(1); //Loosen grip to do the spinney thing with the preloaded cone
        if(Robot.autonClock.get()>=3.30 && Robot.autonClock.get()<3.35)
        BananaClaw.openClaw(0);
        if(Robot.autonClock.get()>=2.35 && Robot.autonClock.get()<2.40)
        BananaDriveStraight.driveStraight(tipToeForward); //this method has a wait time of 1 second, operation time of 1 second
        //Robot.driveTrain.aimPIDState = true;
        if(Robot.autonClock.get()>=4.35 && Robot.autonClock.get()<4.95) //giving 0.6 second for claw to open
        BananaClaw.openClaw(1);
        if(Robot.autonClock.get()>=4.95 && Robot.autonClock.get()<5.0)
        BananaClaw.openClaw(0);
        //Robot.driveTrain.aimPIDState = false;
        if(Robot.autonClock.get()>=5.7 && Robot.autonClock.get()<5.8)//LEFT OF HERRRRRRRRRRRREEEEEEEEEEEEEEEEEE!
        BananaDriveStraight.driveStraight(straightBackwardSide); //this method has a wait time of 1 second
        if(Robot.autonClock.get()>=6.8 && Robot.autonClock.get()<8.2)
        BananaPreSets.cargoPickUp();
        if(Robot.autonClock.get()>=8.2 && Robot.autonClock.get()<9.5)
        BananaTurn.turnPID(leftFullTurn, 0.0);
        if(Robot.autonClock.get()>=9.5 && Robot.autonClock.get()<12.5)
        BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        if(Robot.autonClock.get()>=9.6 && Robot.autonClock.get()<12.0)
        BananaClaw.closeClaw(1.0);
        if(Robot.autonClock.get()>=12.0 && Robot.autonClock.get()<12.1)
        BananaClaw.closeClaw(0.0);
        if(Robot.autonClock.get()>=12.1 && Robot.autonClock.get()<13.0)
        BananaPreSets.lvl1RocketBall();
        if(Robot.autonClock.get()>=13.0 && Robot.autonClock.get()<14.0)
        BananaTurn.turnPID(leftFullTurn, 0.0); //this method has a wait time of 1 second
        if(Robot.autonClock.get()>=14 && Robot.autonClock.get()<14.05)
        BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        
        

    }

    public static void leftScoreMob() //turns right, away from the wall on the left side of the robot
    {
        //BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardSide);
        BananaPreSets.cargoPickUp();
        BananaTurn.turnPID(rightFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl1RocketBall();
        BananaTurn.turnPID(rightFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);

    }

    //turns left, doesn't actually matter which way; also inches onto the charging station
    public static void midScorePark() 
    {
        //BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardMid);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.neutralPivotAngle();
        BananaTurn.turnPID(leftFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardMid);
        BananaDriveStraight.driveStraight(tipToeForward);

    } 
}
