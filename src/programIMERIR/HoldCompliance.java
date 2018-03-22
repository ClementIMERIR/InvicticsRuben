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
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.Motion;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
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
	private ObjectFrame penWorldAlign;
	
	@Inject
	private CartesianImpedanceControlMode freeMode;
	private CartesianImpedanceControlMode drawMode;
	private ForceCondition grabForce;
	private ForceCondition penCollision;
	private ForceCondition penForceZpos;
	private ForceCondition penForceZneg;
	private ICondition penForceZposCond;
	private ICondition penForceZnegCond;
	private ConditionObserver grabForceObserver;
	private ConditionObserver penCollisionObserver;
	private Vector penCollisionForce;
	private Double squareSize;
	private Double altitude;
	
	//Definition de la bar de boutons, et des boutons
	IUserKeyBar buttonsBar;
	IUserKey startDrawing;
	IUserKey resetPTPHome;
	IUserKey goToWorkingPoint;
	
	private double forceGrab = 10;
	private double forcePenCollision = 2;
	private double forceZPos = 1.5;
	private double forceZNeg = 0.3;
	
	/**
	 * Fonction activée lorsqu'une force est appliquée sur le bras pour pouvoir le déplacer
	 * en freeMovement
	 */
	private IRisingEdgeListener grabForceListener = new IRisingEdgeListener() {
		@Override
		public void onRisingEdge(ConditionObserver conditionObserver, Date time,
				int missedEvents) {
			getLogger().info("FreeMovement triggered");
			freeMovementRobot();
		}
	};
	
	/**
	 * Fonction activée lorsque le bouton "startDrawing" est enfoncé
	 */
	private IUserKeyListener startDrawingListener = new IUserKeyListener() {
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			if(event == UserKeyEvent.KeyDown){
				getLogger().info("Button startDrawing triggered");
				Frame currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
				getLogger().info("penInfos = " + currentFrame.getX() + " , " + currentFrame.getY() + " , "  + currentFrame.getZ());
				penCollisionObserver.enable();
				RelativeLIN descente = linRel(0,0,-currentFrame.getZ());
				descente.setCartVelocity(30);
				descente.breakWhen(penCollision);
				penWorldAlign.move(descente);
				//DESSIN
				currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
				altitude = currentFrame.getZ();
				getLogger().info("Altitude = "+altitude);
				//penCollisionObserver.disable();
				//penWorldAlign.move(linRel(200, 0, 0).setJointVelocityRel(40).setMode(drawMode));
				drawSquare(currentFrame.getX(), currentFrame.getY(), squareSize);
				penWorldAlign.move(ptp(getApplicationData().getFrame("/WorkingTable/P6")));
			}
		}
	};
	
	/**
	 * Fonction activée lorsque le bouton "resetPTPHome" est enfoncé
	 */
	private IUserKeyListener resetPTPHomeListener = new IUserKeyListener() {
		
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			if(event == UserKeyEvent.KeyDown){
				robot.move(ptpHome());
				grabForceObserver.disable();
				penCollisionObserver.disable();
			}
		}
	};
	
	/**
	 * Fonction activée lorsque le bouton "goToWorkingPoint" est enfoncé
	 */
	private IUserKeyListener goToWorkingPointListener = new IUserKeyListener() {
		
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			if(event == UserKeyEvent.KeyDown){
				penWorldAlign.move(ptp(getApplicationData().getFrame("/WorkingTable/P6")));
				grabForceObserver.enable();
				penCollisionObserver.disable();
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
			//penCollisionObserver.disable();
			getLogger().info("=============");
			getLogger().info("Pen Collision");
			getLogger().info("=============");

			Frame currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
			getLogger().info("FrameInfo = " + currentFrame.getX() + " , " + currentFrame.getY() + " , "  + currentFrame.getZ());
		}
	};
	
	/**
	 * Cette fonction permet de réguler la position en Z suivant la force sur l'axeZ appliquée à la pointe de l'outil
	 * ->Si la force en Z+ est supérieure à 1.5N, le bras s'élève pour diminuer la force
	 */
	private ICallbackAction adjustZpos = new ICallbackAction() {
		
		@Override
		public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
			penCollisionObserver.disable();
			getLogger().info("Ajustement en Z+");
			penCollisionForce = robot.getExternalForceTorque(penWorldAlign).getForce();
			getLogger().info("penCollisionForce Z: " + penCollisionForce.getZ());
			double deltaZ = penCollisionForce.getZ();
			
			Frame frame = robot.getCurrentCartesianPosition(penWorldAlign);
			getLogger().info("Before getZ : "+frame.getZ());
			frame.setZ(frame.getZ() + deltaZ);
			getLogger().info("After getZ : "+frame.getZ());
			
			penCollisionForce = robot.getExternalForceTorque(penWorldAlign).getForce();
			getLogger().info("penCollisionForce Z: " + penCollisionForce.getZ());
			//penWorldAlign.moveAsync(linRel(0, 0, 2).setMode(drawMode));
			//penWorldAlign.moveAsync(lin(frame));
			penCollisionObserver.enable();
		}
	};
	
	/**
	 * Cette fonction permet de réguler la position en Z suivant la force sur l'axeZ appliquée à la pointe de l'outil
	 * ->Si la force en Z- est supérieure à 0.3N, le bras descend pour augmenter la force
	 */
	private ICallbackAction adjustZneg = new ICallbackAction() {
		
		@Override
		public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
			penCollisionObserver.disable();
			getLogger().info("Ajustement en Z-");
			penCollisionForce = robot.getExternalForceTorque(penWorldAlign).getForce();
			getLogger().info("penCollisionForce Z: " + penCollisionForce.getZ());
			double deltaZ = penCollisionForce.getZ();
			
			Frame frame = robot.getCurrentCartesianPosition(penWorldAlign);
			getLogger().info("Before getZ : "+frame.getZ());
			frame.setZ(frame.getZ() - deltaZ);
			getLogger().info("After getZ : "+frame.getZ());
			
			penCollisionForce = robot.getExternalForceTorque(penWorldAlign).getForce();
			getLogger().info("penCollisionForce Z: " + penCollisionForce.getZ());
			//penWorldAlign.moveAsync(linRel(0, 0, -2));
			penWorldAlign.moveAsync(lin(frame));
			penCollisionObserver.enable();
		}
	};
	
	@Override
	public void initialize() {
		// initialize your application here
		penTCP = pen.getFrame("PenTCP");
		penWorldAlign = pen.getFrame("PenTCP/PenAlignWorld");
		pen.attachTo(robot.getFlange());
		
		//Ajout des boutons pour dessiner/PTPHome/WorkingPoint
		buttonsBar = getApplicationUI().createUserKeyBar("Drawing");
		startDrawing = buttonsBar.addUserKey(0, startDrawingListener, true);
		startDrawing.setText(UserKeyAlignment.MiddleLeft, "Start Drawing");
		resetPTPHome = buttonsBar.addUserKey(1, resetPTPHomeListener, true);
		resetPTPHome.setText(UserKeyAlignment.MiddleLeft, "PTPHome");
		goToWorkingPoint = buttonsBar.addUserKey(2, goToWorkingPointListener, true);
		goToWorkingPoint.setText(UserKeyAlignment.MiddleLeft, "WorkingPoint");
		buttonsBar.publish();
		
		//définition du mode d'impédence pour déplacer le robot à la main
		freeMode = new CartesianImpedanceControlMode();
		freeMode.parametrize(CartDOF.X,CartDOF.Y,CartDOF.Z).setStiffness(10);
		freeMode.parametrize(CartDOF.A,CartDOF.B,CartDOF.C).setStiffness(300);
		
		//définition du mode d'impédence pour le dessins
		drawMode = new CartesianImpedanceControlMode();
		drawMode.parametrize(CartDOF.Z).setStiffness(500);
		
		//Condition de force activée lorsqu'une force supérieure à forceGrab(10N) est détectée pour bouger librement le bras
		grabForce = ForceCondition.createSpatialForceCondition(robot.getFlange(), forceGrab);
		grabForceObserver = getObserverManager().createConditionObserver(grabForce, NotificationType.EdgesOnly,grabForceListener);
		
		//Condition de force activée pour une force supérieure à forcePenCollision(2N) lors d'une collision du marqueur sur une surface
		penCollision = ForceCondition.createNormalForceCondition(penWorldAlign, CoordinateAxis.Z, forcePenCollision);
		penCollisionObserver = getObserverManager().createConditionObserver(penCollision, NotificationType.EdgesOnly,penCollisionListener);
		
		//Force positive en Z activée si la force dépasse forceZPos(1.5N)
		penForceZpos = ForceCondition.createNormalForceCondition(penWorldAlign, CoordinateAxis.Z, forceZPos);
		//Force positive en Z activée si la force dépasse forceZPos(0.3N)
		penForceZneg = ForceCondition.createNormalForceCondition(penWorldAlign, CoordinateAxis.Z, forceZNeg);
		//Condition vraie si penForceZpos est vraie
		penForceZposCond = penForceZpos;
		//Condition vraie si penForceZneg est fausse
		penForceZnegCond = penForceZneg.invert();
	}

	@Override
	public void run() {
		// your application execution starts here
		//penWorldAlign.move(ptp(getApplicationData().getFrame("/WorkingTable/P6")));
		//grabForceObserver.enable();	
		while(true){
			squareSize = getApplicationData().getProcessData("squareSize").getValue();
			
			//getLogger().info("ZForce : "+getZForce(penWorldAlign));
			double zForce = getSumForces(penWorldAlign);
			if(zForce > 2.5){
				//getLogger().info("TROP DE PRESSION!!");
			}else if((zForce < 2) && (zForce > 0.3)){
				//getLogger().info("Bonne pression.");
			}else{
				//getLogger().info("PAS ASSEZ DE PRESSION!!");
			}
			
			ThreadUtil.milliSleep(500);
		}
	}
	
	public double getSumForces(ObjectFrame frame){
		penCollisionForce = robot.getExternalForceTorque(penWorldAlign).getForce();
		//getLogger().info("penCollisionForce X: " + penCollisionForce.getX());
		//getLogger().info("penCollisionForce Y: " + penCollisionForce.getY());
		//getLogger().info("penCollisionForce Z: " + penCollisionForce.getZ());
		double somme = penCollisionForce.getX()+penCollisionForce.getY()+penCollisionForce.getZ();
		//getLogger().info("SommeF : "+somme);
		return somme;
	}
	
	public double getZForce(ObjectFrame frame){
		penCollisionForce = robot.getExternalForceTorque(penWorldAlign).getForce();
		//getLogger().info("penCollisionForce X: " + penCollisionForce.getX());
		//getLogger().info("penCollisionForce Y: " + penCollisionForce.getY());
		getLogger().info("penCollisionForce Z: " + penCollisionForce.getZ());
		double ZForce = penCollisionForce.getZ();
		return ZForce;
	}
	
	/**
	 * Méthode permettant le déplacement du robot à la main
	 */
	public void freeMovementRobot(){
		double sumForces;
		Frame currFrameState;
		do{
			sumForces = displayLogForces(penWorldAlign);
			robot.move(positionHold(freeMode,300, TimeUnit.MILLISECONDS));
		}while(sumForces >= 10);
		
		currFrameState = robot.getCurrentCartesianPosition(penWorldAlign);
		currFrameState.setAlphaRad(Math.toRadians(0));
		currFrameState.setBetaRad(Math.toRadians(0));
		currFrameState.setGammaRad(Math.toRadians(0));
		penWorldAlign.move(ptp(currFrameState));
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
			currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
			force = robot.getExternalForceTorque(penWorldAlign).getForce();
			movedestX = (destX - currentFrame.getX())/accuracy;
			movedestY = (destY - currentFrame.getY())/accuracy;
			moveZ = force.getZ();
			penWorldAlign.move(linRel(movedestX,movedestY,moveZ).setCartVelocity(100));
		}while(currentFrame.getX() != destX && currentFrame.getY() != destY);
	}

	public void drawSquare(double startingPointX, double startingPointY, double dimension){
		getLogger().info("Début du dessin du carré");
		//double altitude = 100;
		//worldAltitude = penWorldAlign.getZ();
		getLogger().info("Altitude = "+altitude);
		
		Frame p0 = new Frame(startingPointX,startingPointY,altitude);
		Frame p1 = new Frame(p0.getX() + dimension,p0.getY(),altitude);
		Frame p2 = new Frame(p0.getX() + dimension, p0.getY() + dimension,altitude);
		Frame p3 = new Frame(p0.getX(), p0.getY() + dimension, altitude);
		getLogger().info("p0:("+p0.getX()+","+p0.getY()+")\n"
				+"p1:("+p1.getX()+","+p1.getY()+")\n"
				+"p2:("+p2.getX()+","+p2.getY()+")\n"
				+"p3:("+p3.getX()+","+p3.getY()+")");
		
		displayLogForces(penWorldAlign);
		penWorldAlign.move(lin(p1).setMode(drawMode).setCartVelocity(100).triggerWhen(penForceZnegCond, adjustZneg));
		
		displayLogForces(penWorldAlign);
		penWorldAlign.move(lin(p2).setMode(drawMode).setCartVelocity(100));
		
		displayLogForces(penWorldAlign);
		penWorldAlign.move(lin(p3).setMode(drawMode).setCartVelocity(100));
		
		displayLogForces(penWorldAlign);
		penWorldAlign.move(lin(p0).setMode(drawMode).setCartVelocity(100));
		
		/**
		//move relatif
		//définition des parametres du déplacement 
		RelativeLIN moveSquareSide = linRel(0,0,0);
		moveSquareSide.setMode(drawMode);
		moveSquareSide.setCartVelocity(100);
		
		displayLogForces(penWorldAlign);

		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		//définition du déplacement de p0 a p1
		moveSquareSide.setXOffset(squareSize);
		moveSquareSide.setYOffset(0);
		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		penWorldAlign.move(moveSquareSide.triggerWhen(penCollision, AdjustZAxis));
		
		displayLogForces(penWorldAlign);
		
		//défintion du déplacement de p1 a p2
		moveSquareSide.setXOffset(0);
		moveSquareSide.setYOffset(squareSize);
		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		penWorldAlign.move(moveSquareSide.triggerWhen(penCollision, AdjustZAxis));
		
		displayLogForces(penWorldAlign);
		
		//défintion du déplacement de p2 a p3		
		moveSquareSide.setXOffset(-squareSize);
		moveSquareSide.setYOffset(0);
		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		penWorldAlign.move(moveSquareSide.triggerWhen(penCollision, AdjustZAxis));
		
		displayLogForces(penWorldAlign);
		
		//défintion du déplacement de p3 a p0
		moveSquareSide.setXOffset(0);
		moveSquareSide.setYOffset(-squareSize);
		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		penWorldAlign.move(moveSquareSide.triggerWhen(penCollision, AdjustZAxis));
		
		displayLogForces(penWorldAlign);
		**/
		//penCollisionObserver.enable();
		
		//avec le move de l'API
//		penTCP.move(lin(p1).setMode(drawMode));
//		penTCP.move(lin(p2).setMode(drawMode));
//		penTCP.move(lin(p3).setMode(drawMode));
//		penTCP.move(lin(p0).setMode(drawMode));
		
		//avec le movePenTo perso
//		movePenTo(p0.getX(), p0.getY());
//		movePenTo(p1.getX(), p1.getY());
//		movePenTo(p2.getX(), p2.getY());
//		movePenTo(p3.getX(), p3.getY());
	}
	
	/** 
	 * Méthode affichant les forces exercées sur la "frame"
	 * @param frame
	 */
	public double displayLogForces(ObjectFrame frame){
		Vector force;
		double sumForces;
		force = robot.getExternalForceTorque(frame).getForce();
		sumForces = Math.abs(force.getX()) + Math.abs(force.getY())	+ Math.abs(force.getZ());
		getLogger().info("Forces : " + force.getX() + " , " + force.getY() + " , " + force.getZ());
		getLogger().info("Somme forces = " + sumForces);
		return sumForces;
	}
}