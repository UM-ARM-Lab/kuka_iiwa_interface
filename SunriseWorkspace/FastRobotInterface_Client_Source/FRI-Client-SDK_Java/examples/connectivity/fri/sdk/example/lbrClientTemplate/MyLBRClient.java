package connectivity.fri.sdk.example.lbrClientTemplate;

import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;

/**
 * Empty client implementation for FRI SDK.
 */
public class MyLBRClient extends LBRClient
{
    /**
     * Constructor.
     */
    MyLBRClient()
    {
    }

    /**
     * Callback for FRI state changes.
     * 
     * @param oldState
     *            the old state
     * @param newState
     *            the new state
     */
    @Override
    public void onStateChange(FRISessionState oldState, FRISessionState newState)
    {

        switch (newState)
        {
        case MONITORING_WAIT:
        {
            break;
        }
        case MONITORING_READY:
        {
            break;
        }
        case COMMANDING_WAIT:
        {
            break;
        }
        case COMMANDING_ACTIVE:
        {
            break;
        }
        default:
        {
            break;
        }
        }
    }

    @Override
    public void monitor()
    {
        super.monitor();

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /***************************************************************************/
    }

    @Override
    public void waitForCommand()
    {
        super.waitForCommand();

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /***************************************************************************/
    }

    @Override
    public void command()
    {
        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /***************************************************************************/

        // In command(), the joint angle values have to be set. 
        //        getRobotCommand().setJointPosition(newJointValues);
    }
}
