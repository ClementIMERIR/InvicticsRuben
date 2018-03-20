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
	private CartesianImpedanceControlMode drawMode;
	private ForceCondition grabForce;
	private ForceCondition penCollision;
	private ConditionObserver grabForceObserver;
	private ConditionObserver penCollisionObserver;
	private ForceSensorData data;
	private ForceSensorData penCollisionForce;
	private Double squareSize;
	
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
				getLogger().info("penInfos = " + currentFrame.getX() + " , " + currentFrame.getY() + " , "  + currentFrame.getZ());
				//penTCP.copyWithRedundancy(robot.getFrame("/WorkingTable/StartingPoint"));
				grabForceObserver.disable();
				penCollisionObserver.enable();
				//drawSquare(currentFrame.getX(), currentFrame.getY(), squareSize);
				penTCP.move(linRel(0,0,currentFrame.getZ()));
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
			penCollisionObserver.disable();
			getLogger().info("Pen Collision");
			Frame currentFrame = robot.getCurrentCartesianPosition(penTCP);
			drawSquare(currentFrame.getX(), currentFrame.getY(), squareSize);
		}
	};
	
	@Override
	public void initialize() {
		// initialize your application here
		penTCP = pen.getFrame("PenTCP");
		pen.attachTo(robot.getFlange());
		
		//Ajout d'un bouton pour lancer le dessin
		buttonsBar = getApplicationUI().createUserKeyBar("Drawing");
		startDrawing = buttonsBar.addUserKey(0, startDrawingListener, true);
		startDrawing.setText(UserKeyAlignment.MiddleLeft, "Start Drawing");
		buttonsBar.publish();
		
		//définition du mode d'impédence pour déplacer le robot à la main
		freeMode = new CartesianImpedanceControlMode();
		freeMode.parametrize(CartDOF.X,CartDOF.Y,CartDOF.Z).setStiffness(10);
		freeMode.parametrize(CartDOF.A,CartDOF.B,CartDOF.C).setStiffness(5000);
		
		//définition du mode d'impédence pour le dessins
		drawMode = new CartesianImpedanceControlMode();
		drawMode.parametrize(CartDOF.ALL).setStiffness(5);
		
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
		penTCP.move(ptp(getApplicationData().getFrame("/WorkingTable/WaitingPoint")));
		grabForceObserver.enable();	
		while(true){
			ThreadUtil.milliSleep(1000);
		}
	}
	
	/**
	 * Méthode permettant le déplacement du robot à la main
	 */
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
		
		currFrameState = robot.getCurrentCartesianPosition(penTCP);
		currFrameState.setAlphaRad(Math.toRadians(-180));
		currFrameState.setBetaRad(Math.toRadians(0));
		currFrameState.setGammaRad(Math.toRadians(180));
		penTCP.move(ptp(currFrameState));
		
		//pen.getFrame("/Pen/PenTCP").copyWithRedundancy(robot.getFrame("/WorkingTable/StartingPoint"));
	}
	
	/**
	 * Méthode qui déplace le pen au point "destination" avec correction des z en fonction des forces
	 * @param destX
	 * @param destY
	 */
	public void movePenTo(double destX, double destY){
		Frame currentFrame;
		Vector force;
		double movedestX, movedestY, moveZ, accuracy = 1000;
		do{
			currentFrame = robot.getCurrentCartesianPosition(penTCP);
			force = robot.getExternalForceTorque(penTCP).getForce();
			movedestX = (destX - currentFrame.getX())/accuracy;
			movedestY = (destY - currentFrame.getY())/accuracy;
			moveZ = force.getZ();
			penTCP.move(linRel(movedestX,movedestY,moveZ));
		}while(currentFrame.getX() != destX && currentFrame.getY() != destY);
	}
	
	public void drawSquare(double startingPointX, double startingPointY, double dimension){
		getLogger().info("Début du dessin du carré");
		double altitude = 100;
		Frame p0 = new Frame(startingPointX,startingPointY,altitude);
		Frame p1 = new Frame(p0.getX() + dimension,p0.getY(),altitude);
		Frame p2 = new Frame(p0.getX() + dimension, p0.getY() + dimension,altitude);
		Frame p3 = new Frame(p0.getX(), p0.getY() + dimension, altitude);
		getLogger().info("p0:("+p0.getX()+","+p0.getY()+") "
				+"p1:("+p1.getX()+","+p1.getY()+")"
				+"p2:("+p2.getX()+","+p2.getY()+")"
				+"p3:("+p3.getX()+","+p3.getY()+")");
		
		//avec le move de l'API
		penTCP.move(lin(p1).setMode(drawMode));
		penTCP.move(lin(p2).setMode(drawMode));
		penTCP.move(lin(p3).setMode(drawMode));
		penTCP.move(lin(p0).setMode(drawMode));
		
		//avec le movePenTo perso
//		movePenTo(p0.getX(), p0.getY());
//		movePenTo(p1.getX(), p1.getY());
//		movePenTo(p2.getX(), p2.getY());
//		movePenTo(p3.getX(), p3.getY());
	}
}