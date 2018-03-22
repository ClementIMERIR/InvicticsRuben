package programIMERIR;

import java.sql.*;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.PositionInformation;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

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
public class TrainingLegs_final extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	@Inject
	@Named("LegLift")
	private Tool legLift;// Création d'un objet outil
	@Inject
	@Named("Leg")
	private Workpiece leg;
	@Inject
	@Named("Leg1k5")
	private Workpiece leg1k5;
	@Inject
	@Named("Leg_halima")
	private Workpiece leg_halima;
	@Inject
	@Named("Leg_thomas")
	private Workpiece leg_thomas;
	@Inject
	@Named("Leg_mathis")
	private Workpiece leg_mathis;

	private CartesianImpedanceControlMode mode;
	private IUserKeyBar gripperBar;
	private IUserKey openKey;
	private Frame firedCurrPos;
	// variable accessible via process data
	private Integer tempo, nbcycle;
	private Double angle;
	private Double vitesse;

	// ---------------------
	private int answer;
	private int answer2;
	private String nom;
	private String URL;
	private String login;
	private String password;
	private String sql;
	private Connection connection;
	private Statement stmt;
	private ResultSet resultat;
	private String current_nom;
	private int current_id;

	private enum Personne {
		halima, thomas, mathis;
	}

	@Override
	public void initialize() {
		legLift.attachTo(robot.getFlange());// "Fixation" de l'outil à la bride
											// du robot.
		nom = getApplicationData().getProcessData("name").getValue();
		gripperBar = getApplicationUI().createUserKeyBar("Gripper");
		openKey = gripperBar.addDoubleUserKey(2, myfunction, false);
		mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.ALL).setStiffness(10.0);
		// mode.parametrize(CartDOF.ALL).setDamping(0.7);
		answer = -1; // initialize a une valeur nom utilisable par la boite de
						// dialogue
		answer2 = -1;
		URL = "jdbc:mysql://172.31.1.66/imerir";
		login = "imerir";
		password = "";
		connection = null;
		stmt = null;
		resultat = null;
		current_id = -1;
		// se connecter a la base de données
		try {
			Class.forName("com.mysql.jdbc.Driver");
			connection = DriverManager.getConnection(URL, login, password);
			// interaction avec la base
			getLogger().info("nom:" + nom);
			// identifier le patient en utilisant sont nom.
			sql = "SELECT * FROM infos_patients WHERE nom = \'" + nom + "\'";
			stmt = connection.createStatement();
			resultat = stmt.executeQuery(sql);
			/* Récupération des données du résultat de la requête de lecture */
			while (resultat.next()) {
				current_id = resultat.getInt("id_patient");
				current_nom = resultat.getString("prenom");
			}
			// récuperer les parametres du patient en utilisant son id.
			sql = "SELECT * FROM parametres_patients WHERE id_patient = \'"
					+ current_id + "\'";
			stmt = connection.createStatement();
			resultat = stmt.executeQuery(sql);
			/* Récupération des données du résultat de la requête de lecture */
			while (resultat.next()) {
				tempo = resultat.getInt("tempo");
				nbcycle = resultat.getInt("nb_cycles");
				angle = resultat.getDouble("angle");
				vitesse = resultat.getDouble("vitesse_ang");
			}
		} catch (SQLException sqle) {
			answer = getApplicationUI().displayModalDialog(
					ApplicationDialogType.ERROR, "erreur connection", "Ok");
			sqle.printStackTrace();
		} catch (ClassNotFoundException e) {
			answer = getApplicationUI().displayModalDialog(
					ApplicationDialogType.ERROR,
					"erreur ClassNotFoundException", "Ok");
			e.printStackTrace();
		} finally {
			try {
				System.out.println("le nom est : " + current_nom);
				connection.close();
				stmt.close();
			} catch (SQLException e) {
				e.printStackTrace();
			}
		}
		answer = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Bonjour Mme/Mr : " + nom + " " + current_nom, "Ok");
		answer = -1;
	}

	// fonction appellée lors du click sur btn 0.
	IUserKeyListener myfunction = new IUserKeyListener() {
		@Override
		public void onKeyEvent(IUserKey key, UserKeyEvent event) {
			// quand le boutton est clicker bouger le robot en cartesien.
			if (event == UserKeyEvent.FirstKeyDown) {
				getLogger().info("FirstKeyDown appuyé");
				while(event != UserKeyEvent.SecondKeyDown){
				robot.move(positionHold(mode, 1, TimeUnit.SECONDS));
				}
				getLogger().info("KeyDown appuyé");
				firedCurrPos = robot.getCurrentCartesianPosition(legLift.getDefaultMotionFrame());
				getLogger().info(firedCurrPos.toString());
			} 
		}
	};

	@Override
	public void run() {
		// / debug des variables
		getLogger().info(tempo.toString());
		getLogger().info(nbcycle.toString());
		getLogger().info(angle.toString());
		getLogger().info(vitesse.toString());
		// PARTIE EN COMMUN POUR TOUS LES PATIENTS------------------------
		robot.move(ptpHome().setJointVelocityRel(0.5));

		answer2 = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Quel jambe voulez vous entrainer ?", "Droite", "Gauche",
				"STOP");

		while (answer2 != 2) {
			if (answer2 == 0) {
				legLift.getFrame("/TCP").move(
						ptp(getApplicationData().getFrame("/Genoux/P1")));
			} else {
				legLift.getFrame("/TCP").move(
						ptp(getApplicationData().getFrame("/Genoux/P2")));
			}
			answer2 = -1;

			while (answer != 0) {
				ThreadUtil.milliSleep(5000);// attache la jambe
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"La jambe de Mme/Mr : " + nom + " est elle en place ?",
						"Oui", "Non");
			}
			answer = -1;
			// FIN PARTIE EN COMMUN-------------------------------------------
			// personnaliser les mouvements pour chaque patient existant
			Personne personne = Personne.valueOf(current_nom);
			switch (personne) {
			case halima:
				leg_halima.getFrame("/PNP_enfant_halima").attachTo(
						legLift.getFrame("TCP"));
				while (answer != 1) {
					for (int i = 0; i < nbcycle; i++) {
						leg_halima.getFrame("Genoux").move(
								linRel(0, 0, 0, Math.toRadians(-angle), 0, 0)
										.setCartVelocity(vitesse));
						leg_halima.getFrame("Genoux").move(
								linRel(0, 0, 0, Math.toRadians(angle), 0, 0)
										.setCartVelocity(vitesse));
					}
					answer = getApplicationUI().displayModalDialog(
							ApplicationDialogType.QUESTION,
							"Voulez vous refaire un cycle sur cette jambe?",
							"Oui", "Non");
				}
				answer = -1;
				ThreadUtil.milliSleep(tempo);// 10 sec pour détacher sa jambe
				break;

			case thomas:
				// leg_thomas.getFrame("/PNP_enfant_thomas").attachTo(legLift.getFrame("/Dummy/PNP_parent"));
				leg_thomas.getFrame("/PNP_enfant_thomas").attachTo(
						legLift.getFrame("TCP"));
				// robot.setSafetyWorkpiece(leg1k5); //déclare en sécurité
				while (answer != 1) {
					for (int i = 0; i < nbcycle; i++) {
						leg_thomas.getFrame("Genoux").move(
								linRel(0, 0, 0, Math.toRadians(-angle), 0, 0)
										.setCartVelocity(vitesse));
						leg_thomas.getFrame("Genoux").move(
								linRel(0, 0, 0, Math.toRadians(angle), 0, 0)
										.setCartVelocity(vitesse));
					}
					answer = getApplicationUI().displayModalDialog(
							ApplicationDialogType.QUESTION,
							"Voulez vous refaire un cycle sur cette jambe?",
							"Oui", "Non");
				}
				answer = -1;
				ThreadUtil.milliSleep(tempo);// 10 sec pour détacher sa jambe
				break;
			case mathis:
				leg_mathis.getFrame("/PNP_enfant_mathis").attachTo(
						legLift.getFrame("TCP"));
				while (answer != 1) {
					for (int j = 0; j < nbcycle; j++) {
						leg_mathis.getFrame("Genoux").move(
								linRel(0, 0, 0, Math.toRadians(-angle), 0, 0)
										.setCartVelocity(vitesse));
						leg_mathis.getFrame("Genoux").move(
								linRel(0, 0, 0, Math.toRadians(angle), 0, 0)
										.setCartVelocity(vitesse));
					}
					answer = getApplicationUI().displayModalDialog(
							ApplicationDialogType.QUESTION,
							"Voulez vous refaire un cycle sur cette jambe ?",
							"Oui", "Non");
				}
				answer = -1;
				ThreadUtil.milliSleep(tempo);// 10 sec pour détacher sa jambe
				break;
			default:
				getApplicationUI()
						.displayModalDialog(ApplicationDialogType.ERROR,
								"Le nom séléctionné n'existe pas, pensez à écrire le nom en majuscule");
				break;
			}
			answer2 = getApplicationUI()
					.displayModalDialog(
							ApplicationDialogType.QUESTION,
							"Si vous voulez faire une nouvelle jambe, séléctionner le côté ou mettez STOP ?",
							"Droite", "Gauche", "STOP");
		}

		// AVANT DE SORTIR DEMANDER SI LA JAMBE EST ENLEVEE PUIS REMETTRE LE
		// BRAS EN POS INITIALE.
		while (answer != 0) {
			ThreadUtil.milliSleep(5000);// détache la jambe
			answer = getApplicationUI().displayModalDialog(
					ApplicationDialogType.QUESTION,
					"La jambe de Mme/Mr : " + nom + " est elle enlevée ?",
					"Oui", "Non");
		}
		answer = -1;

		leg.detach();// detache la jambe de l'outil en logiciel
		robot.move(ptpHome().setJointVelocityRel(0.5));
		/*
		 * if (nom.equals("Pierre")){ tempo =
		 * getApplicationData().getProcessData("tempo").getValue(); nbcycle =
		 * getApplicationData().getProcessData("nbcycle").getValue(); angle =
		 * getApplicationData().getProcessData("angle").getValue(); vitesse =
		 * getApplicationData().getProcessData("vitesse").getValue();
		 * 
		 * // your application execution starts here
		 * robot.move(ptpHome().setJointVelocityRel(0.5));
		 * legLift.getFrame("/Dummy/PNP_parent"
		 * ).move(ptp(getApplicationData().getFrame("/Genoux/P1"))); //message
		 * variable
		 * answeranswer=getApplicationUI().displayModalDialog(ApplicationDialogType
		 * .QUESTION, "La jambe du patient est elle en place ?", "Oui","Non");
		 * while(answer != 0){ ThreadUtil.milliSleep(5000);//attache la jambe
		 * answer
		 * =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION
		 * , "La jambe de "+nom+" est elle en place ?", "Oui","Non"); } answer =
		 * -1;
		 * leg.getFrame("/PNP_enfant").attachTo(legLift.getFrame("/Dummy/PNP_parent"
		 * )); while(answer !=1){ for(int i=0;i<nbcycle;i++){
		 * leg.getFrame("Genoux"
		 * ).move(linRel(0,0,0,Math.toRadians(-angle),0,0).setCartVelocity
		 * (vitesse));
		 * leg.getFrame("Genoux").move(linRel(0,0,0,Math.toRadians(angle
		 * ),0,0).setCartVelocity(vitesse)); }
		 * answer=getApplicationUI().displayModalDialog
		 * (ApplicationDialogType.QUESTION, "Voulez vous refaire un cycle ?",
		 * "Oui","Non"); } answer = -1; ThreadUtil.milliSleep(tempo);//10 sec
		 * pour détacher sa jambe while(answer != 0){
		 * ThreadUtil.milliSleep(5000);//détache la jambe
		 * answer=getApplicationUI
		 * ().displayModalDialog(ApplicationDialogType.QUESTION,
		 * "La jambe de "+nom+" est elle enlevé ?", "Oui","Non"); } answer = -1;
		 * leg.detach();//detache la jambe de l'outil en logiciel
		 * robot.move(ptpHome().setJointVelocityRel(0.5)); }
		 * 
		 * else if (nom.equals("Paul")){ tempo =
		 * getApplicationData().getProcessData("tempo").getValue(); nbcycle =
		 * getApplicationData().getProcessData("nbcycle").getValue(); angle =
		 * getApplicationData().getProcessData("angle").getValue(); vitesse =
		 * getApplicationData().getProcessData("vitesse").getValue();
		 * 
		 * // your application execution starts here
		 * robot.move(ptpHome().setJointVelocityRel(0.5));
		 * legLift.getFrame("/Dummy/PNP_parent"
		 * ).move(ptp(getApplicationData().getFrame("/Genoux/P1"))); //message
		 * variable
		 * answeranswer=getApplicationUI().displayModalDialog(ApplicationDialogType
		 * .QUESTION, "La jambe du patient est elle en place ?", "Oui","Non");
		 * while(answer != 0){ ThreadUtil.milliSleep(5000);//attache la jambe
		 * answer
		 * =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION
		 * , "La jambe de "+nom+" est elle en place ?", "Oui","Non"); } answer =
		 * -1; leg1k5.getFrame("/PNP_enfant").attachTo(legLift.getFrame(
		 * "/Dummy/PNP_parent")); while(answer !=1){ for(int i=0;i<nbcycle;i++){
		 * leg1k5
		 * .getFrame("Genoux").move(linRel(0,0,0,Math.toRadians(-angle),0,0
		 * ).setCartVelocity(vitesse));
		 * leg1k5.getFrame("Genoux").move(linRel(0,0
		 * ,0,Math.toRadians(angle),0,0).setCartVelocity(vitesse)); }
		 * answer=getApplicationUI
		 * ().displayModalDialog(ApplicationDialogType.QUESTION,
		 * "Voulez vous refaire un cycle ?", "Oui","Non"); } answer = -1;
		 * ThreadUtil.milliSleep(tempo);//10 sec pour détacher sa jambe
		 * while(answer != 0){ ThreadUtil.milliSleep(5000);//détache la jambe
		 * answer
		 * =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION
		 * , "La jambe de "+nom+" est elle enlevé ?", "Oui","Non"); } answer =
		 * -1; leg1k5.detach();//detache la jambe de l'outil en logiciel
		 * robot.move(ptpHome().setJointVelocityRel(0.5)); } else if
		 * (nom.equals("Jack")){ tempo =
		 * getApplicationData().getProcessData("tempo").getValue(); nbcycle =
		 * getApplicationData().getProcessData("nbcycle").getValue(); angle =
		 * getApplicationData().getProcessData("angle").getValue(); vitesse =
		 * getApplicationData().getProcessData("vitesse").getValue(); }
		 */
	}
}