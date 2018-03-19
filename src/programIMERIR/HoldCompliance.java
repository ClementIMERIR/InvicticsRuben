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
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
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
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
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
	private ObjectFrame penTCP;
	
	@Inject
	private CartesianImpedanceControlMode freeMode;
	private ForceCondition grabForce;
	private ForceCondition penCollision;
	private ConditionObserver grabForceObserver;
	private ConditionObserver penCollisionObserver;
	private ForceSensorData data;
	private ForceSensorData penCollisionForce;
	
	IUserKeyBar buttonsBar;
	IUserKey startDrawing;
	
	/**
	 * Fonction activée lorsqu'une force est appliquée sur le bras pour pouvoir le déplacer
	 * en freeMovement
	 */
	private IRisingEdgeListener grabForceListener = new IRisingEdgeListener() {
		@Override
		public void onRisingEdge(ConditionObserver conditionObserver, Date time,
				int missedEvents) {
			// Méthode appeler lorsque une force plus fote a 10 N est appliquée
			getLogger().info("FreeMovement triggered");
			freeMovementRobot();
		}
	};
	
	/**
	 * Fonction activée lorsque le bouton startDrawing est enfoncé
	 */
	private IUserKeyListener startDrawingListener = new IUserKeyListener() {
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			if(event == UserKeyEvent.KeyDown){
				getLogger().info("Button startDrawing triggered");
				Frame currentFrame = robot.getCurrentCartesianPosition(penTCP);
				getLogger().info("Avant copy penInfos = " + penTCP.getX() + " , " + penTCP.getY() + " , "  + penTCP.getZ());
				penTCP.copyWithRedundancy(robot.getFrame("/WorkingTable/StartingPoint"));
				//draw();
				penTCP.move(linRel(currentFrame.getX(), currentFrame.getY(), 0));
			}
		}
	};
	
	/**
	 * Fonction activée lorsqu'une collision du stylo est détectée
	 */
	private IRisingEdgeListener penCollisionListener = new IRisingEdgeListener() {
		@Override
		public void onRisingEdge(ConditionObserver conditionObserver, Date time,
				int missedEvents) {
			getLogger().info("Pen Collision");
		}
	};
	
	@Override
	public void initialize() {
		// initialize your application here
		penTCP = pen.getFrame("PenTCP");
		pen.attachTo(robot.getFlange());
		
		buttonsBar = getApplicationUI().createUserKeyBar("Drawing");
		startDrawing = buttonsBar.addUserKey(0, startDrawingListener, true);
		startDrawing.setText(UserKeyAlignment.MiddleLeft, "Start Drawing");
		buttonsBar.publish();
		
		freeMode = new CartesianImpedanceControlMode();
		freeMode.parametrize(CartDOF.X,CartDOF.Y,CartDOF.Z).setStiffness(10);
		freeMode.parametrize(CartDOF.A,CartDOF.B,CartDOF.C).setStiffness(5);
		//Condition de force activée lorsqu'une force supérieure à 10N est détectée pour bouger librement le bras
		grabForce = ForceCondition.createSpatialForceCondition(robot.getFlange(), 10);
		grabForceObserver = getObserverManager().createConditionObserver(grabForce, NotificationType.EdgesOnly,grabForceListener);
		//Condition de force activée pour une force supérieure à 2N lors d'une collision du marqueur sur une surface
		penCollision = ForceCondition.createSpatialForceCondition(penTCP, 2);
		penCollisionObserver = getObserverManager().createConditionObserver(penCollision, NotificationType.EdgesOnly,penCollisionListener);
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
			getLogger().info("Forces : " + force.getX() + " , " + force.getY() + " , " + force.getZ());
			getLogger().info("Somme forces = " + sumForces);
			robot.move(positionHold(freeMode,300, TimeUnit.MILLISECONDS));
		}while(sumForces >= 10);
		
		currFrameState = robot.getCurrentCartesianPosition(robot.getFlange());
		currFrameState.setAlphaRad(Math.toRadians(-180));
		currFrameState.setBetaRad(Math.toRadians(0));
		currFrameState.setGammaRad(Math.toRadians(180));
		robot.move(ptp(currFrameState));
		
		//pen.getFrame("/Pen/PenTCP").copyWithRedundancy(robot.getFrame("/WorkingTable/StartingPoint"));
	}
}