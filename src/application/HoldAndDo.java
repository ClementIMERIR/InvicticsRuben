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
 * @author Alexandre
 * @version 1.6
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
		ArrayList<ObjectFrame> sortedByX = framePoints;
		
		//trier par x <
		for (int i = 0; i < sortedByX.size(); i++) {
			for (int j = sortedByX.size() - 1; j > i; j--) {
				if (sortedByX.get(i).getX() > sortedByX.get(j).getX()) {
					ObjectFrame tmp = sortedByX.get(i);
					sortedByX.set(i, sortedByX.get(j)) ;
					sortedByX.set(j,tmp);
				}
			}
		}
		
		int index = 0;
		for (int i = 0; i < framePoints.size(); i++) {
			index = framePoints.get(i).getX() == sortedByX.get(0).getX() ? i: index;
		}
		
		double deltaXStart = sortedByX.get(1).getX() - sortedByX.get(0).getX();
		double deltaYStart = sortedByX.get(1).getY() - sortedByX.get(0).getY();
		double incXStart = largeurOutil * ( deltaXStart / ( Math.abs(deltaYStart) + Math.abs(deltaXStart) ) );
		double incYStart = largeurOutil * ( deltaYStart / ( Math.abs(deltaYStart) + Math.abs(deltaXStart) ) );
		
//		double deltaXEnd = sortedByX.get(3).getX() - sortedByX.get(2).getX();
//		double deltaYEnd = sortedByX.get(3).getY() - sortedByX.get(2).getY();
//		double incXEnd = largeurOutil * ( deltaXEnd / ( Math.abs(deltaYEnd) + Math.abs(deltaXEnd) ) );
//		double incYEnd = largeurOutil * ( deltaYEnd / ( Math.abs(deltaYEnd) + Math.abs(deltaXEnd) ) );

		ObjectFrame thirdFrame = sortedByX.get(0).getY() < sortedByX.get(1).getY() ? sortedByX.get(3) : sortedByX.get(2);
		
		double deltaXPolish = thirdFrame.getX() - sortedByX.get(0).getX();
		double deltaYPolish = thirdFrame.getY() - sortedByX.get(0).getY();
		double incXPolish = deltaXPolish / ( Math.abs(deltaYPolish) + Math.abs(deltaXPolish) );
		double incYPolish = deltaYPolish / ( Math.abs(deltaYPolish) + Math.abs(deltaXPolish) );
		
		//Ponçage
		robot.move(ptp(jointPositions.get(index)));
		getLogger().info("En place sur le 1er point");
		for(int i = 0 ; i < deltaYStart ; i += largeurOutil/10){
			robot.move(linRel(deltaXPolish, deltaYPolish, 0.0).setJointVelocityRel(0.5));
			robot.move(linRel(-deltaXPolish, -deltaYPolish, 0.0).setJointVelocityRel(0.5));
			
			pliers.getFrame("Sander").move(linRel(incXStart, incYStart, 0.0).setJointVelocityRel(0.5));
			getLogger().info("1 aller-retour terminé");
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