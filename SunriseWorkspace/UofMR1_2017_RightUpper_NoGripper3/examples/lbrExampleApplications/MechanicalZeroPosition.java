package lbrExampleApplications;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class MechanicalZeroPosition extends RoboticsAPIApplication {
	private LBR lbr;
	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot will move to the mechanical zero position.";

	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	public void run() {
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to the mechanical zero position");
		PTP ptpToMechanicalZeroPosition = ptp(0,0,0,0,0,0,0);
		ptpToMechanicalZeroPosition.setJointVelocityRel(0.25);
		lbr.move(ptpToMechanicalZeroPosition);
	}


}
