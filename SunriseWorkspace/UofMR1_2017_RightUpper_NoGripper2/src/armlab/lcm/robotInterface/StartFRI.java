// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package armlab.lcm.robotInterface;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 *
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class StartFRI extends RoboticsAPIApplication {
	private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    PositionHold posHold;
    FRIJointOverlay jointOverlay;

    private static final String CLIENT_IP = "192.170.11.2"; // the IP of the machine running Ubuntu22 / ROS2 for the network interface connected to KONI
    private static final int PORT_ID = 30201;
	private static final int TS = 5; //in ms

    IFRISessionListener listener = new IFRISessionListener(){
    	@Override
    	public void onFRIConnectionQualityChanged(
    	FRIChannelInformation friChannelInformation){
    	getLogger().info("QualityChangedEvent - quality:" +
    	friChannelInformation.getQuality()+"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	@Override
    	public void onFRISessionStateChanged(
    	FRIChannelInformation friChannelInformation){
    	getLogger().info("SessionStateChangedEvent - session state:" +
    	friChannelInformation.getFRISessionState() +"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	};

	@Override
	public void initialize() {
		_lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        _clientName = CLIENT_IP;
	}


	@Override
	public void run() {
        // Select the type of control
		String ques = "Select FRI control mode  :\n";
    	double res = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,ques , "POSITION","TORQUE","MONITORING","CARTESIAN","Cancel");

		if(res == 0){
			PositionControlMode ctrMode = new PositionControlMode();
			posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
		}
		else if (res == 1 || res == 2){
			JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(0,0,0,0,0,0,0);
			ctrMode.setDampingForAllJoints(0.7);
			ctrMode.setStiffness(600, 600, 300, 300, 100, 100, 50);
			posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
		}
		else if (res == 3){
			// TODO: set up parameters
			CartesianImpedanceControlMode ctrMode = new CartesianImpedanceControlMode();
			// CartesianImpedanceControlMode.CartImpBuilder parametrize(CartDOF... degreeOfFreedom
			posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
		}
		else return;
		
		// configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
		friConfiguration.setPortOnRemote(PORT_ID);
        friConfiguration.setSendPeriodMilliSec(TS);
        friConfiguration.setReceiveMultiplier(2);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName() + ":" + friConfiguration.getPortOnRemote());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms");

        FRISession friSession = new FRISession(friConfiguration);
        friSession.addFRISessionListener(listener);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(600, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        if(res == 0){
        	getLogger().info("POSITION mode");
	    	jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.POSITION);
		}
		else if (res == 1){
			getLogger().info("TORQUE mode");
			jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.TORQUE);
		}
		else if(res== 2){
			getLogger().info("NO_COMMAND mode");
			jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.NO_COMMAND_MODE);
		}
		else {
			return;
		}
		
        _lbr.move(posHold.addMotionOverlay(jointOverlay));
        
        // done
        friSession.close();
        getLogger().info("FRI connection closed.");
        getLogger().info("Application stopped.");
	}

	public static void main(final String[] args)
    {
        final StartFRI app = new StartFRI();
        app.runApplication();
    }
}
