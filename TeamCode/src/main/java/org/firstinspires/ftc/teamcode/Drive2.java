package org.firstinspires.ftc.teamcode;

/**
 * Created by howard on 12/21/17.
 */

public class Drive2 {
    public static final float MAX_TURN_TIME = (float)4000;
    public static final float BUMP_TIME     = (float)2000;

    public botMotors fwd6(float pwr, long goal, long stageTime){
        botMotors mPower = new botMotors();
        if (goal < stageTime){
            mPower.leftFront = pwr;
            mPower.rightFront = pwr;
            mPower.leftRear = pwr;
            mPower.rightRear = pwr;
            mPower.status = -1;
        }
        else
        {
            mPower.leftFront = 0;
            mPower.rightFront = 0;
            mPower.leftRear = 0;
            mPower.rightRear = 0;
            mPower.status = 0;
        }
        return mPower;
    }
    /***********************************************************
     * gyroturn5
     *  startHeading  - input. heading when this "state" started
     *  currHeading   - input. what is the heading when gt5 invoked
     *  newHeading    - input. Destination heading
     *  turnPwr       - input. -100 to 100, percent power. negative means counterclockwise
     *  turnTime      - input. how long this "state" has been in play
     * @return pwrSet - multiply the pwrSet by -1 for the starboard motor in the calling routine.
     *   Note: The gyroturn5 will quit after MAX_TURN_TIME, and will give a "bump" to increase turn
     *   power after BUMP_TIME.  The idea is to juice the power
     */
    public botMotors gyroTurn5(int startHeading, int currHeading, int newHeading, int turnPwr, float turnTime){
        botMotors mPower = new botMotors();
        float pwrSet = turnPwr;
        int accumTurn = Math.abs(startHeading - currHeading);
        accumTurn = (accumTurn > 360)? (360-accumTurn):accumTurn;
        int cw = (turnPwr < 0)? -1: 1;
        int transit = (((currHeading > newHeading) && (cw > 0)) ||
                ((currHeading < newHeading) && (cw < 0))) ?  360 : 0;
        int desiredRotation = Math.abs(transit + (cw*newHeading) + ((-1*cw)*currHeading));
        desiredRotation = (desiredRotation > 360) ? desiredRotation - 360 : desiredRotation;
        if((accumTurn < desiredRotation) && (turnTime < MAX_TURN_TIME)){
            pwrSet = (turnTime > BUMP_TIME)? (float)turnPwr: (float)1.0;
        }
        else
        {
            pwrSet = 0;
        }
        return mPower;
    }

    /*****************************************************
     * Fwd5 - a simplified forward Drive for autonomous
     * double traveled
     * double goal
     * float pwr
     * double duration
     * @return - commanded power.  Do this once for each motor
     */


    /***********************************************************
     **          calculate necessary turn amount              **
     **********************************************************/
    public static int getNeededTurn(int isNow, int want, int cw) {
        cw = (cw < 0) ? -1 : 1;
        int transit = (((isNow > want) && (cw > 0)) ||
                ((isNow < want) && (cw < 0))) ?  360 : 0;
        int neededTurn = Math.abs(transit + (cw*want) + ((-1*cw)*isNow));
        neededTurn = (neededTurn > 360) ? neededTurn - 360 : neededTurn;
        return (neededTurn);
    }




}
