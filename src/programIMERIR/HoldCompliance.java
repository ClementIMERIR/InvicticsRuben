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
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.Motion;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.HandGuidingControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.requestModel.GetCurrentConfigurationRequest;
import com.kuka.roboticsAPI.requestModel.SetManualOverrideRequest;
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
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

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
	private ConditionObserver grabForceObserver;
	private ConditionObserver penCollisionObserver;
	private ForceSensorData penCollisionForce;
	private Double squareSize;
	private double moveZGlobal;
	private RelativeLIN globalMove;
	private int drawingMethode;
	private double step;
	
	IUserKeyBar buttonsBar;
	IUserKey startDrawingCompliant;
	IUserKey startDrawingZCalc;
	
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
			step = -1;
			freeMovementRobot();
		}
	};
	
	/**
	 * Fonction activée lorsque le bouton startDrawing est enfoncé
	 */
	private IUserKeyListener startDrawingCompliantListener = new IUserKeyListener() {
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			if(event == UserKeyEvent.KeyDown){
				squareSize = getApplicationData().getProcessData("squareSize").getValue();
				getLogger().info("Button startDrawing triggered");
				Frame currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
				getLogger().info("penInfos = " + currentFrame.getX() + " , " + currentFrame.getY() + " , "  + currentFrame.getZ());
//				//grabForceObserver.disable();
				penCollisionObserver.enable();
				RelativeLIN descente = linRel(0,0,-currentFrame.getZ());
				descente.setCartVelocity(30);
				descente.breakWhen(penCollision);
				drawingMethode = 0;
				penWorldAlign.move(descente);
			}
		}
	};
	
	/**
	 * Fonction activée lorsque le bouton Drawing z calculation est enfoncé
	 */
	private IUserKeyListener startDrawingZCalcListener = new IUserKeyListener() {
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			if(event == UserKeyEvent.KeyDown){
				squareSize = getApplicationData().getProcessData("squareSize").getValue();
				getLogger().info("Button startDrawing triggered");
				Frame currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
				getLogger().info("penInfos = " + currentFrame.getX() + " , " + currentFrame.getY() + " , "  + currentFrame.getZ());
//				//grabForceObserver.disable();
				penCollisionObserver.enable();
				RelativeLIN descente = linRel(0,0,-currentFrame.getZ());
				descente.setCartVelocity(30);
				descente.breakWhen(penCollision);
				drawingMethode = 1;
				penWorldAlign.move(descente);
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
			Frame currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
			getLogger().info("SquareSize = " + squareSize);
			getLogger().info("FrameInfo = " + currentFrame.getX() + " , " + currentFrame.getY() + " , "  + currentFrame.getZ());
			if(drawingMethode == 0){
				drawSquareWithCompliance(currentFrame.getX(), currentFrame.getY(), squareSize);
//				drawCircleCompliance(currentFrame.getX(), currentFrame.getY(), squareSize/2);
			}else{
				drawSquareZCalc(currentFrame.getX(), currentFrame.getY(), squareSize);
			}
			penWorldAlign.move(linRel(0,0,10));
			penWorldAlign.move(ptp(getApplicationData().getFrame("/WorkingTable/P6")));
			}
	};
	
	@Override
	public void initialize() {
		// initialize your application here
		penTCP = pen.getFrame("PenTCP");
		penWorldAlign = pen.getFrame("PenTCP/PenAlignWorld");
		pen.attachTo(robot.getFlange());
		squareSize = getApplicationData().getProcessData("squareSize").getValue();
		
		//Ajout d'un bouton pour lancer le dessin avec le mode copliant
		buttonsBar = getApplicationUI().createUserKeyBar("Drawing");
		startDrawingCompliant = buttonsBar.addUserKey(0, startDrawingCompliantListener, true);
		startDrawingCompliant.setText(UserKeyAlignment.MiddleLeft, "Start Drawing Compliant");
		
		//Ajout d'un bouton pour lancer le dessin avec recalcul des z manuel
		startDrawingZCalc = buttonsBar.addUserKey(1, startDrawingZCalcListener, true);
		startDrawingZCalc.setText(UserKeyAlignment.MiddleLeft, "Start Drawing Z Calc");
		buttonsBar.publish();
				
		
		//définition du mode d'impédence pour déplacer le robot à la main
		freeMode = new CartesianImpedanceControlMode();
		freeMode.parametrize(CartDOF.X,CartDOF.Y,CartDOF.Z).setStiffness(10);
		freeMode.parametrize(CartDOF.A,CartDOF.B,CartDOF.C).setStiffness(300);
		
		//définition du mode d'impédence pour le dessins
		drawMode = new CartesianImpedanceControlMode();
		drawMode.parametrize(CartDOF.Z).setStiffness(150);
		drawMode.parametrize(CartDOF.Z).setAdditionalControlForce(-0.5);
		drawMode.parametrize(CartDOF.Z).setDamping(0.1);
		
		//Condition de force activée lorsqu'une force supérieure à 10N est détectée pour bouger librement le bras
		grabForce = ForceCondition.createSpatialForceCondition(robot.getFlange(), 15);
		grabForceObserver = getObserverManager().createConditionObserver(grabForce, NotificationType.EdgesOnly,grabForceListener);
		
		//Condition de force activée pour une force supérieure à 2N lors d'une collision du marqueur sur une surface
		penCollision = ForceCondition.createNormalForceCondition(penWorldAlign,CoordinateAxis.Z, 1);
		penCollisionObserver = getObserverManager().createConditionObserver(penCollision, NotificationType.EdgesOnly,penCollisionListener);
	}

	@Override
	public void run() {
		// your application execution starts here
		penWorldAlign.move(ptp(getApplicationData().getFrame("/WorkingTable/P6")));
		grabForceObserver.enable();	
		//robot.move(handGuiding());
		Vector forces;
		while(true){
//			forces = robot.getExternalForceTorque(penWorldAlign).getForce();
//			moveZGlobal = -(forces.getZ()-1);
			ThreadUtil.milliSleep(1000);
		}
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
		double movedestX, movedestY, moveZ, accuracy = 300;
		double sumForces;
		step = accuracy;
		getLogger().info("dest:("+destX+","+destY+")");
		currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
		getLogger().info("currentFrame:("+currentFrame.getX()+","+currentFrame.getY()+")");
		movedestX = (Math.floor(destX) - Math.floor(currentFrame.getX()))/accuracy;
		movedestY = (Math.floor(destY) - Math.floor(currentFrame.getY()))/accuracy;
		getLogger().info("move:("+movedestX+","+movedestY+")");
		do{
			currentFrame = robot.getCurrentCartesianPosition(penWorldAlign);
			getLogger().info("currentFrame:("+currentFrame.getX()+","+currentFrame.getY()+")");
			force = robot.getExternalForceTorque(penWorldAlign).getForce();
			sumForces = Math.abs(force.getX()) + Math.abs(force.getY()) + Math.abs(force.getZ());
			displayLogForces(penWorldAlign);
			if(force.getZ() <= 0){
				moveZ = force.getZ();
			}else{
//				moveZ = Math.log(force.getZ());
				moveZ = (sumForces-2)/10;
			}
//			moveZ = (force.getZ()-1)/3;
			getLogger().info("moveZ= "+moveZ);
			penWorldAlign.move(linRel(movedestX,movedestY,moveZ));
			step--;
		}while(step > 0);
	}
	
	public void drawSquareZCalc(double startingPointX, double startingPointY, double dimension){
		getLogger().info("Début du dessin du carré");
		double altitude = 100;
		Frame p0 = new Frame(startingPointX,startingPointY,altitude);
		Frame p1 = new Frame(p0.getX() + dimension,p0.getY(),altitude);
		Frame p2 = new Frame(p0.getX() + dimension, p0.getY() + dimension,altitude);
		Frame p3 = new Frame(p0.getX(), p0.getY() + dimension, altitude);
		getLogger().info("p0:("+p0.getX()+","+p0.getY()+")\n"
				+"p1:("+p1.getX()+","+p1.getY()+")\n"
				+"p2:("+p2.getX()+","+p2.getY()+")\n"
				+"p3:("+p3.getX()+","+p3.getY()+")");
			
		//avec le movePenTo perso
		movePenTo(p1.getX(), p1.getY());
		movePenTo(p2.getX(), p2.getY());
		movePenTo(p3.getX(), p3.getY());
		movePenTo(p0.getX(), p0.getY());
	}
	
	public void drawSquareWithCompliance(double startingPointX, double startingPointY, double dimension){
		getLogger().info("Début du dessin du carré");
		double altitude = 100;
		Frame p0 = new Frame(startingPointX,startingPointY,altitude);
		Frame p1 = new Frame(p0.getX() + dimension,p0.getY(),altitude);
		Frame p2 = new Frame(p0.getX() + dimension, p0.getY() + dimension,altitude);
		Frame p3 = new Frame(p0.getX(), p0.getY() + dimension, altitude);
		getLogger().info("p0:("+p0.getX()+","+p0.getY()+")\n"
				+"p1:("+p1.getX()+","+p1.getY()+")\n"
				+"p2:("+p2.getX()+","+p2.getY()+")\n"
				+"p3:("+p3.getX()+","+p3.getY()+")");
		
		//move relatif
		//définition des parametres du déplacement
		getLogger().info("début dessin avec linRel");
		RelativeLIN moveSquareSide = linRel(0,0,-3).breakWhen(grabForce);
		moveSquareSide.setMode(drawMode);
		moveSquareSide.setCartVelocity(50);
		
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
		penWorldAlign.move(moveSquareSide);
		
		displayLogForces(penWorldAlign);
		
		//défintion du déplacement de p1 a p2
		moveSquareSide.setXOffset(0);
		moveSquareSide.setYOffset(squareSize);
		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		penWorldAlign.move(moveSquareSide);
		
		displayLogForces(penWorldAlign);
		
		//défintion du déplacement de p2 a p3		
		moveSquareSide.setXOffset(-squareSize);
		moveSquareSide.setYOffset(0);
		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		penWorldAlign.move(moveSquareSide);
		
		displayLogForces(penWorldAlign);
		
		//défintion du déplacement de p3 a p0
		moveSquareSide.setXOffset(0);
		moveSquareSide.setYOffset(-squareSize);
		getLogger().info("moveSquare : (" + moveSquareSide.getOffset().getX()+","
				+moveSquareSide.getOffset().getY()+","
				+moveSquareSide.getOffset().getZ()+")");
		penWorldAlign.move(moveSquareSide);
		
		displayLogForces(penWorldAlign);
		
//		//avec le move de l'API
//		getLogger().info("début dessin avec lin");
//		LIN moveTop1 = lin(p1);
//		moveTop1.setMode(drawMode);
//		moveTop1.setCartVelocity(100);
//		
//		LIN moveTop2 = lin(p2);
//		moveTop2.setMode(drawMode);
//		moveTop2.setCartVelocity(100);
//		
//		LIN moveTop3 = lin(p3);
//		moveTop3.setMode(drawMode);
//		moveTop3.setCartVelocity(100);
//		
//		LIN moveTop0 = lin(p0);
//		moveTop1.setMode(drawMode);
//		moveTop1.setCartVelocity(100);
//		
//		penWorldAlign.move(moveTop1);
//		penWorldAlign.move(moveTop2);
//		penWorldAlign.move(moveTop3);
//		penWorldAlign.move(moveTop0);
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
		getLogger().info("Somme forces (absolue) = " + sumForces);
		return sumForces;
	}
	
	public void drawCircleCompliance(double startingPointX, double startingPointY, double rayon){
		getLogger().info("Début du dessin du carré");
		double altitude = 530;
		Frame p0 = new Frame(startingPointX,startingPointY,altitude);
		Frame p1 = new Frame(p0.getX() + rayon,p0.getY() - rayon,altitude);
		Frame p2 = new Frame(p0.getX() + rayon*2, p0.getY(),altitude);
		Frame p3 = new Frame(p0.getX() + rayon, p0.getY() + rayon, altitude);
		getLogger().info("p0:("+p0.getX()+","+p0.getY()+")\n"
				+"p1:("+p1.getX()+","+p1.getY()+")\n"
				+"p2:("+p2.getX()+","+p2.getY()+")\n"
				+"p3:("+p3.getX()+","+p3.getY()+")");

//		//avec le move de l'API
		penWorldAlign.move(circ(p1,p2).setMode(drawMode).setCartVelocity(50));
		penWorldAlign.move(circ(p3,p0).setMode(drawMode).setCartVelocity(50));
	}
}