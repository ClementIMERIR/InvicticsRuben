package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
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
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

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

	private CartesianImpedanceControlMode mode;
//	private JointImpedanceControlMode mode;
	private double[] jointPosition;
	private JointPosition jPosition;
	
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
	private ArrayList<JointPosition> jointPositions = null;
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

					moving = false;//Stop compliant mode
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
		
		jointPositions = new ArrayList<JointPosition>();
		
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

		double minX = 9999;
		double minY = 9999;
		double maxX = 0;
		double maxY = 0;
		double distanceX = 0;
		double distanceY = 0;
		
		double deltaX = 0;
		double deltaY = 0;
		int startingPointIndex = 0;//The one that has the tiniest X
		
		ObjectFrame refFrame1 = framePoints.get(0);//Plus petit X, point de reference.
		ObjectFrame refFrame2 = framePoints.get(1);//Deuxieme plus petit X.
		ObjectFrame refFrame3 = framePoints.get(2);//Plus grande distance (diagonale).
		ObjectFrame refFrame4 = framePoints.get(3);//Le point non utilisé

		for(int i = 0; i < framePoints.size() ; i++){
			ObjectFrame point = framePoints.get(i);
			
			//Prendre le point avec le plus petit X
			if(point.getX() < minX){
				refFrame1 = point;
				minX = point.getX();
				startingPointIndex = i;
			}
			
			minY = point.getY() < minY ? point.getY() : minY;
			
			maxX = point.getX() > minX ? point.getX() : minX;
			maxY = point.getY() > minY ? point.getY() : minY;
			
			refFrame3 = refFrame1.distanceTo(point) > refFrame1.distanceTo(refFrame3) ? point : refFrame3;
		}
		
		distanceX = maxX - minX;
		distanceY = maxY - minY;

		//Recherche du deuxième plus petit X
		for (ObjectFrame point : framePoints) {
			if(point.distanceTo(refFrame1) != 0){
				refFrame2 = Math.abs( point.getX() - refFrame1.getX() ) < Math.abs(refFrame2.getX() - refFrame1.getX() ) ? point : refFrame2;
			}
			
			//On enregistre le point qui n'a pas été utilisé
			refFrame4 = point.getX() != refFrame1.getX() && point.getX() != refFrame2.getX() && point.getX() != refFrame3.getX() ? point : refFrame4;
		}
		deltaX = refFrame2.getX() - refFrame1.getX();
		deltaY = refFrame2.getY() - refFrame1.getY();
		double tiniestDelta = Math.abs(deltaX) > Math.abs(deltaY) ? deltaY : deltaX;
		double greatestDelta = Math.abs(deltaX) > Math.abs(deltaY) ? deltaX : deltaY;
		double divDecalage = tiniestDelta / ( tiniestDelta + greatestDelta );
		double distanceToDo = refFrame1.distanceTo(refFrame4);
		
		jPosition = jointPositions.get(startingPointIndex);
		robot.move(ptp( jPosition.get(0), jPosition.get(1), jPosition.get(2), jPosition.get(3), jPosition.get(4), jPosition.get(5), jPosition.get(6)).setJointVelocityRel(0.5));
		
		for(double i = 0 ; i < distanceToDo ; i += largeurOutil) {
			pliers.getFrame("Sander").move(linRel(deltaX, deltaY, 0).setJointVelocityRel(0.5));
			pliers.getFrame("Sander").move(linRel(-deltaX, -deltaY, 0).setJointVelocityRel(0.5));
			
			if(tiniestDelta == deltaX){
				pliers.getFrame("Sander").move(linRel(largeurOutil * divDecalage, largeurOutil * (1 - divDecalage), 0).setJointVelocityRel(0.5));				
			}else {
				pliers.getFrame("Sander").move(linRel(largeurOutil * (1 - divDecalage), largeurOutil * divDecalage, 0).setJointVelocityRel(0.5));
			}
		}

		/*-------------------------------------------------------------------------------------------------------------------*/
		
		polishKey.setLED(UserKeyAlignment.MiddleLeft, UserKeyLED.Red, UserKeyLEDSize.Small);
		getLogger().info("Ponçage terminé.");
	}
	
	/**
	 * Register the current state as a position.
	 */
	private void registerPosition(){
		getLogger().info(new StringBuilder("Enregistrement de la position ").append(currentPointIndex+1).append("...").toString());

		currentPointIndex++;
		
		jointPositions.add(robot.getCurrentJointPosition());
		
		//parameters
		String pointNameString = new StringBuilder("NP").append(String.valueOf(currentPointIndex)).toString();//NP1,NP2,NP3,NP4.
		SceneGraphObject owner = pliers;
		
		Frame toolVector = robot.getPositionInformation(pliers.getFrame("/Sander")).getCurrentCartesianPosition();

		ITransformation transformation = XyzAbcTransformation.ofTranslation(toolVector.getX(), toolVector.getY(), toolVector.getZ());
		
		ITransformationProvider transformationProvider = new StaticTransformationProvider(transformation);
		ObjectFrame parent = getApplicationData().getFrame("/Workspace");
		ObjectFrame newPointFrame = new ObjectFrame(pointNameString, parent , owner, transformationProvider);

		framePoints.set(currentPointIndex - 1, newPointFrame);
		
		currentPointIndex = currentPointIndex == 4 ? 1 : currentPointIndex;
		
		getLogger().info("Enregistrement de la position terminé");
	}
}

/**
 * Saves
 * //Calculer la distance entre ce point, et les 3 autres points pour éliminer la diagonale
	//Entre les points restants, récupérer celui avec le second x le plus petit
	
	//deltaX = X deuxième point - X premier point
	//deltaY = Y deuxième point - Y premier point
	//Le ponçage incrémentera de deltaX et deltaY dans le mouvement aller, et le décrémentera dans le retour
	//Ensuite, test si deltaX ou deltaY est le plus grand (retenir qui est le plus grand) puis faire le plus petit sur le plus grand
	//deltaPetit/(deltaPetit + deltaGrand) = un pourcentage de 0 à 1/2
	//cela nous donnera le rapport pour le déplacement de l'outil pour continuer le ponçage
	//deltaPetit (soit l'incrément X soit l'incrément Y) utilisera le pourcentage obtenu dans le déplacement de l'outil en faisant : largeurOutil x résultat du rapport
	//deltaGrand (soit l'incrément X soit l'incrément Y) utilisera le pourcentage obtenu dans le déplacement de l'outil en faisant : largeurOutil x (1- résultat du rapport)
	
	//ensuite récupérer le point non utilisé qui n'est pas le point de la diagonale
	//calculer la distance entre notre origine (le point avec le plus petit x) et celui ci pour avoir la longueur a garder dans la boucle for
	//cela nous donnera la valeur a ne pas dépasser pour le (i = 0 ; i < distance Origine jusqu'au point concerné ; i+= largeurOutil)
 */