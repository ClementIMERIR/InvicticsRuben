package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ITransformationProvider;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.SceneGraphObject;
import com.kuka.roboticsAPI.geometricModel.StaticTransformationProvider;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.persistenceModel.templateModel.TemplateElement;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
import com.sun.org.apache.xalan.internal.xsltc.compiler.Template;

/**
 * 
 * @author Clément Bourdarie
 */
public class HoldAndDo extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	@Inject
	@Named("Pliers")
	private Tool pliers;
	private ArrayList<JointPosition> alJPositions = new ArrayList<JointPosition>();
	private CartesianImpedanceControlMode mode;
//	private JointImpedanceControlMode mode;
	private double[] jointPosition;
	
	private IUserKeyBar buttonBar;
	private IUserKey allowMovementKey;
	private IUserKey polishKey;
	private IUserKey registerPositionKey;
	private IUserKey stopApplicationKey;

	private boolean moving = false;
	private boolean finished = false;
	
	private int currentPointIndex;//the index of the point being registered
	
	//Variables polishing
	private ArrayList<ObjectFrame> framePoints;
	double largeurOutil = 60;
	
	@Override
	public void initialize() {
		pliers.attachTo(robot.getFlange());//"Fixation" de l'outil à la bride du robot.
		
		mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.ALL).setStiffness(10);
		
//		mode = new JointImpedanceControlMode(10, 10, 10, 10, 10, 10, 1);
//		mode.setStiffness(10, 10, 10, 10, 10, 10, 1);
		
		//The listener of the allowMovementKey button
		IUserKeyListener moveButtonListener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if(event.equals(UserKeyEvent.KeyUp)){
					moving = !moving;// Allow the robot to move, or stop it.
					key.setText(UserKeyAlignment.MiddleLeft, moving ? "ON" : "OFF");// If the robot was moving show OFF, else show ON.
				}
			}
		};
		
		//The listener of the polishKey button
		IUserKeyListener polishButtonListener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if(event.equals(UserKeyEvent.KeyUp)){
					polishKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Green, UserKeyLEDSize.Small);

					polish();

					polishKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
				}
			}
		};
		
		//The listener of the polishKey button
		IUserKeyListener registerButtonListener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if(event.equals(UserKeyEvent.KeyUp)){
					registerPositionKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);

					registerPosition();

					registerPositionKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
				}
			}
		};
		
		//The listener of the stopApplicationKey button
		IUserKeyListener stopApplicationButtonListener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if(event.equals(UserKeyEvent.KeyUp)){
					getLogger().info("Programme terminé.");
					finished = true;
				}
			}
		};
		
		//The container of the button we're going to create
		buttonBar = getApplicationUI().createUserKeyBar("Mouvement");
		
		//Button allowing the user to move the robot
		allowMovementKey = buttonBar.addUserKey(0, moveButtonListener, true);
		allowMovementKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
		
		//Button starting the polishing
		polishKey = buttonBar.addUserKey(1, polishButtonListener, true);
		polishKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);

		//Button to register a position
		registerPositionKey = buttonBar.addUserKey(2, registerButtonListener, true);
		registerPositionKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
		
		//Button allowing the user to stop the application
		stopApplicationKey = buttonBar.addUserKey(3, stopApplicationButtonListener, true);
		stopApplicationKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
		
		//Make the buttons bar visible
		buttonBar.publish();
		
		framePoints = new ArrayList<ObjectFrame>(){ /**
			 * 
			 */
			private static final long serialVersionUID = 9072283436281698039L;

		{ add(null); add(null); add(null); add(null); } };
		
		currentPointIndex = 0;
	}

	/**
	 * Major function
	 */
	@Override
	public void run() {
		//While the user hasn't pressed the stopApplicationKey
		while(!finished){
			//If the user has pressed the allowMovementKey
			if(moving){
				allowMovement();
			}
		}
	}
	
	/**
	 *  Allow the user to freely move the robot and select 4 points
	 */
	private void allowMovement(){
		//Make the allowMovementKey LED go RED
		getLogger().info("The robot is compliant.");
		allowMovementKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Green, UserKeyLEDSize.Small);
		//Make the robot compliant according to "mode" which makes his stiffness low
		while (moving) {
			robot.move(positionHold(mode, 1, TimeUnit.SECONDS));
			jointPosition = robot.getCurrentJointPosition().get();// Register the current position
		}
		//Make the allowMovementKey LED go RED
		getLogger().info("Stop touching the robot.\nThe robot is not compliant anymore.");
		allowMovementKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
		
		//Move to the registered position so that the robot holds it
		robot.move(ptp(jointPosition));
	}
	
	/**
	 * Make the robot polish the area between the 4 points
	 */
	private void polish(){
		getLogger().info("Ponçage...");
		polishKey.setText(UserKeyAlignment.MiddleLeft, "Ponçage...");
		polishKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Green, UserKeyLEDSize.Small);

		/*-----------------------------TODO make the polishing function--------------------------------------------------------*/
				
		ObjectFrame refFirstPoint = framePoints.get(0);
		ObjectFrame refSecondPoint = framePoints.get(1);
		ObjectFrame refThirdPoint = framePoints.get(2);
		ObjectFrame refDiagPoint = framePoints.get(3);
		int index = 0;
		int index2 = 0;
		int index3 = 0;
		int index4 = 0;
		
		getLogger().info("Recherche du plus petit X");
		
		//Recherche du plus petit X et attribution de son index
		for(int i = 0; i < framePoints.size(); i++){
			refFirstPoint = refFirstPoint.getX() < framePoints.get(i).getX() ? refFirstPoint : framePoints.get(i);
			index = refFirstPoint != framePoints.get(i) ? index : i;
		}
		
		for (int l = 0; l < framePoints.size(); l++) {
			if(l != index)
			{
				refDiagPoint = refFirstPoint.distanceTo(framePoints.get(l)) > refFirstPoint.distanceTo(refDiagPoint) ? framePoints.get(l) : refDiagPoint;
				index4 = refDiagPoint != framePoints.get(l) ? index4 : l;
			}
		}
		
		getLogger().info("Recherche du second X");
		
		//Recherche du second X
		for (int j = 0; j < framePoints.size(); j++) {
			if(j != index && j != index4)
			{
				refSecondPoint = refSecondPoint.getX() < framePoints.get(j).getX() ? refSecondPoint : framePoints.get(j);
			}
			index2 = refSecondPoint != framePoints.get(j) ? index2 : j;
		}
		
		getLogger().info("Recherche du troisième X");
		
		//Recherche du troisième X pas dans la diagonale
		for (int k = 0; k < framePoints.size(); k++) {
			if(k != index && k != index2 && k != index4)
			{
				refThirdPoint = refThirdPoint.getX() < framePoints.get(k).getX() ? refThirdPoint: framePoints.get(k);
				index3 = k;
			}	
		}

		getLogger().info("Calcul des deltas");
		
		//Calcul des deltas entre les différents points
		double deltaX = refSecondPoint.getX() - refFirstPoint.getX();
		double deltaY = refSecondPoint.getY() - refFirstPoint.getY();
		double deltaX2 = refThirdPoint.getX() - refFirstPoint.getX();
		double deltaY2 = refThirdPoint.getY() - refFirstPoint.getY();
		
		getLogger().info("Paramètres");
		
		//Attribution des paramètres pour le passage
		double varX = refFirstPoint.getX();
		//Attribution du décalage à effectuer
		double decalage = 4;
		
		getLogger().info("Déplacement du robot");
		
		//On replace au point avec le plus petit X
		JointPosition JPosition = alJPositions.get(index);
		JointPosition JPosition2 = alJPositions.get(index2);
		JointPosition JPosition3 = alJPositions.get(index3);
		JointPosition JPosition4 = alJPositions.get(index4);
		
		robot.move(ptp(JPosition4.get(0),JPosition4.get(1),JPosition4.get(2),JPosition4.get(3),JPosition4.get(4),JPosition4.get(5),JPosition4.get(6)).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition3.get(0),JPosition3.get(1),JPosition3.get(2),JPosition3.get(3),JPosition3.get(4),JPosition3.get(5),JPosition3.get(6)).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition2.get(0),JPosition2.get(1),JPosition2.get(2),JPosition2.get(3),JPosition2.get(4),JPosition2.get(5),JPosition2.get(6)).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		//Test1 - Parcours possible de toutes les positions pour aller du point 1 au point 2
		getLogger().info("Test de tous les déplacements possibles");
		getLogger().info("pliers avec DeltaX Positif");
		
		pliers.getFrame("Sander").move(linRel(deltaX, deltaY, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		pliers.getFrame("Sander").move(linRel(deltaX, -deltaY, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		//getLogger().info("pliers avec DeltaX Négatif");
		
		//pliers.getFrame("Sander").move(linRel(-deltaX, deltaY, 0).setJointVelocityRel(0.5));
		//robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		//pliers.getFrame("Sander").move(linRel(-deltaX, -deltaY, 0).setJointVelocityRel(0.5));
		//robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		getLogger().info("pliers avec DeltaY En Premier et DeltaX Positif");
		
		pliers.getFrame("Sander").move(linRel(deltaY, deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		pliers.getFrame("Sander").move(linRel(-deltaY, deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		getLogger().info("pliers avec DeltaY En Premier et DeltaX Négatif");
		
		pliers.getFrame("Sander").move(linRel(deltaY, -deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		pliers.getFrame("Sander").move(linRel(-deltaY, -deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		getLogger().info("Robot avec DeltaX Positif");
		
		robot.move(linRel(deltaX, deltaY, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		robot.move(linRel(deltaX, -deltaY, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		getLogger().info("Robot avec DeltaX Négatif");
		
		robot.move(linRel(-deltaX, deltaY, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		robot.move(linRel(-deltaX, -deltaY, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		getLogger().info("Robot avec DeltaY En Premier et DeltaX Positif");
		
		robot.move(linRel(deltaY, deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		robot.move(linRel(-deltaY, deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		getLogger().info("Robot avec DeltaY En Premier et DeltaX Négatif");
		
		robot.move(linRel(deltaY, -deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		robot.move(linRel(-deltaY, -deltaX, 0).setJointVelocityRel(0.5));
		robot.move(ptp(JPosition.get(0),JPosition.get(1),JPosition.get(2),JPosition.get(3),JPosition.get(4),JPosition.get(5),JPosition.get(6)).setJointVelocityRel(0.5));
		
		
		/*getLogger().info("Do While");
		//Tant que le point en diagonale n'est pas atteint
		//Aller-retour puis décalage 
		do{
			pliers.getFrame("Sander").move(linRel(deltaX, deltaY, 0).setJointVelocityRel(0.5));
			pliers.getFrame("Sander").move(linRel(-deltaX, -deltaY, 0).setJointVelocityRel(0.5));
			pliers.getFrame("Sander").move(linRel(deltaX2 / decalage, deltaY2 / decalage, 0).setJointVelocityRel(0.5));
			//Incrémentation de la variable d'arrêt
			varX += deltaX2 / decalage;
		}while(varX < refThirdPoint.getX());*/
		

		/*-------------------------------------------------------------------------------------------------------------------*/
		
		polishKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
		getLogger().info("Ponçage terminé.");
	}
	
	/**
	 * Register the current state as a position.
	 */
	private void registerPosition(){
		getLogger().info(
				new StringBuilder("Enregistrement de la position ").append(currentPointIndex+1).append("...").toString());

		currentPointIndex++;
		alJPositions.add(robot.getCurrentJointPosition());
		
		//parameters
		String pointNameString = 
				new StringBuilder("NP").append(String.valueOf(currentPointIndex)).toString();//NP1,NP2,NP3,NP4.
		SceneGraphObject owner = pliers;
		
		Frame toolVector = robot.getPositionInformation(pliers.getFrame("/Sander")).getCurrentCartesianPosition();

		ITransformation transformation = XyzAbcTransformation.ofTranslation(toolVector.getX(), 
				toolVector.getY(), toolVector.getZ());
		
		ITransformationProvider transformationProvider = new StaticTransformationProvider(transformation);
		ObjectFrame parent = getApplicationData().getFrame("/Workspace");
		ObjectFrame newPointFrame = new ObjectFrame(pointNameString, parent , owner, transformationProvider);

		framePoints.set(currentPointIndex - 1, newPointFrame);
		
		currentPointIndex = currentPointIndex == 4 ? 0 : currentPointIndex;

		getLogger().info("Enregistrement de la position terminé");
	}
}