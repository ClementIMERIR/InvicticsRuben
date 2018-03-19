package programIMERIR;


import java.util.Date;
import java.util.concurrent.TimeUnit;
import java.math.*;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.Motion;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.requestModel.GetCurrentConfigurationRequest;
import com.kuka.roboticsAPI.sensorModel.CartesianPositionInformation;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.sun.org.apache.xerces.internal.parsers.AbstractDOMParser;
import com.sun.xml.internal.bind.v2.runtime.reflect.Lister;
import com.kuka.roboticsAPI.conditionModel.*;
import com.kuka.roboticsAPI.geometricModel.math.*;

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
public class HoldCompliance extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	
	@Inject
	@Named("Pen")
	private Tool pen;
	
	@Inject
	private CartesianImpedanceControlMode freeMode;
	private ForceCondition grabForce;
	private ForceCondition penCollision;
	private ConditionObserver grabForceObserver;
	private ConditionObserver penCollisionObserver;
	private ForceSensorData data;
	private ForceSensorData penCollisionForce;
	private boolean moving = false;
	
	IUserKeyBar buttonsBar;
	IUserKey startDrawing;
	
	
	private IRisingEdgeListener grabForceListener = new IRisingEdgeListener() {
		@Override
		public void onRisingEdge(ConditionObserver conditionObserver, Date time,
				int missedEvents) {
			// M�thode appeler lorsque une force plus fote a 10 N est appliqu�e
			getLogger().info("Action triggered");
			freeMovementRobot();
		}
	};
	
	private IUserKeyListener startDrawingListener = new IUserKeyListener() {
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			
			//key.setLED(UserKeyAlignment.MiddleLeft, moving ? UserKeyLED.Green : UserKeyLED.Red, UserKeyLEDSize.Small);
		}
	};
	
	private IRisingEdgeListener penCollisionListener = new IRisingEdgeListener() {
		@Override
		public void onRisingEdge(ConditionObserver conditionObserver, Date time,
				int missedEvents) {
			// M�thode appeler lorsque une force plus fote a 2 N est appliqu�e
			getLogger().info("Pen Collision");
			
		}
	};
	
	@Override
	public void initialize() {
		// initialize your application here
		freeMode = new CartesianImpedanceControlMode();
		freeMode.parametrize(CartDOF.X,CartDOF.Y,CartDOF.Z).setStiffness(10);
		freeMode.parametrize(CartDOF.A,CartDOF.B,CartDOF.C).setStiffness(5);
		//Condition de force activ�e lorsqu'une force sup�rieure � 10N est d�tect�e
		grabForce = ForceCondition.createSpatialForceCondition(robot.getFlange(), 10);
		grabForceObserver = getObserverManager().createConditionObserver(grabForce, NotificationType.EdgesOnly,grabForceListener);
		penCollision = ForceCondition.createSpatialForceCondition(robot.getFrame("/Pen/TCP"), 2);
		penCollisionObserver = getObserverManager().createConditionObserver(penCollision, NotificationType.EdgesOnly,penCollisionListener);
		pen.attachTo(robot.getFlange());
	}

	@Override
	public void run() {
		// your application execution starts here
		robot.move(ptp(getApplicationData().getFrame("/WorkingTable/WaitingPoint")));
		grabForceObserver.enable();		
		while(true){
			ThreadUtil.milliSleep(1000);
		}
	}
	
	public void freeMovementRobot(){
		Vector force;
		double sumForces;
		Frame currFrameState;
		do{
			data = robot.getExternalForceTorque(robot.getFlange());
			force = data.getForce();
			sumForces = Math.abs(force.getX()) + Math.abs(force.getY())	+ Math.abs(force.getZ());
			getLogger().info("forces : " + force.getX() + " , " + force.getY() + " , " + force.getZ());
			getLogger().info("Somme forces = " + sumForces);
			robot.move(positionHold(freeMode,300, TimeUnit.MILLISECONDS));
		}while(sumForces >= 10);
		
		currFrameState = robot.getCurrentCartesianPosition(robot.getFlange());
		currFrameState.setAlphaRad(Math.toRadians(-180));
		currFrameState.setBetaRad(Math.toRadians(0));
		currFrameState.setGammaRad(Math.toRadians(180));
		
		getLogger().info("Sortie du while moving freely");
		robot.move(ptp(currFrameState));
	}
}