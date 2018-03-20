package programIMERIR;

import java.sql.*;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

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
public class TrainingLegs extends RoboticsAPIApplication {
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

	// variable accessible via process data
	private Integer tempo, nbcycle;
	private Double angle;
	private Double vitesse;
	private String name;

	// ---------------------
	private int answer;
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
		answer = -1; // initialize a une valeur nom utilisable par la boite de
						// dialogue
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
			sql = "SELECT * FROM infos_patients WHERE nom = \'" + nom
					+ "\'";
			stmt = connection.createStatement();
			resultat = stmt.executeQuery(sql);
			/* Récupération des données du résultat de la requête de lecture */
			while (resultat.next()) {
				current_id = resultat.getInt("id_patient");
				current_nom = resultat.getString("prenom");
			}
			// récuperer les parametres du patient en utilisant son id.
			sql = "SELECT * FROM parametres_patients WHERE id_patient = \'" + current_id + "\'";
			stmt = connection.createStatement();
			resultat = stmt.executeQuery(sql);
			/* Récupération des données du résultat de la requête de lecture */
			while (resultat.next()) {				
				tempo = resultat.getInt("tempo");
				nbcycle = resultat.getInt("nb_cycles");
				angle = resultat.getDouble("angle");
				vitesse = resultat.getDouble("vitesse");
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
				"bonjour Mm/Mr : " + nom + current_nom, "Ok");

	}

	@Override
	public void run() {
		// choix nom
		/*
		 * tempo = getApplicationData().getProcessData("tempo").getValue();
		 * nbcycle = getApplicationData().getProcessData("nbcycle").getValue();
		 * angle = getApplicationData().getProcessData("angle").getValue();
		 * vitesse = getApplicationData().getProcessData("vitesse").getValue();
		 */// debug des variables
		getLogger().info(tempo.toString());
		getLogger().info(nbcycle.toString());
		getLogger().info(angle.toString());
		getLogger().info(vitesse.toString());

		Personne personne = Personne.valueOf(current_nom); // surround with try/catch

		switch (personne) {
		case halima:
			// your application execution starts here
			robot.move(ptpHome().setJointVelocityRel(0.5));
			legLift.getFrame("/Dummy/PNP_parent").move(
					ptp(getApplicationData().getFrame("/Genoux/P1")));
			// message variable
			// answeranswer=getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
			// "La jambe du patient est elle en place ?", "Oui","Non");
			while (answer != 0) {
				ThreadUtil.milliSleep(5000);// attache la jambe
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"La jambe de " + nom + " est elle en place ?", "Oui",
						"Non");
			}
			answer = -1;
			leg.getFrame("/PNP_enfant").attachTo(
					legLift.getFrame("/Dummy/PNP_parent"));
			while (answer != 1) {
				for (int i = 0; i < nbcycle; i++) {
					leg.getFrame("Genoux").move(
							linRel(0, 0, 0, Math.toRadians(-angle), 0, 0)
									.setCartVelocity(vitesse));
					leg.getFrame("Genoux").move(
							linRel(0, 0, 0, Math.toRadians(angle), 0, 0)
									.setCartVelocity(vitesse));
				}
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"Voulez vous refaire un cycle ?", "Oui", "Non");
			}
			answer = -1;
			ThreadUtil.milliSleep(tempo);// 10 sec pour détacher sa jambe
			while (answer != 0) {
				ThreadUtil.milliSleep(5000);// détache la jambe
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"La jambe de " + nom + " est elle enlevée ?", "Oui",
						"Non");
			}
			answer = -1;
			leg.detach();// detache la jambe de l'outil en logiciel
			robot.move(ptpHome().setJointVelocityRel(0.5));
			break;

		case thomas:
			// your application execution starts here
			robot.move(ptpHome().setJointVelocityRel(0.5));
			legLift.getFrame("/Dummy/PNP_parent").move(
					ptp(getApplicationData().getFrame("/Genoux/P1")));
			// message variable
			// answer=getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
			// "La jambe du patient est elle en place ?", "Oui","Non");
			while (answer != 0) {
				ThreadUtil.milliSleep(5000);// attache la jambe
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"La jambe de " + nom + " est elle en place ?", "Oui",
						"Non");
			}
			answer = -1;
			leg1k5.getFrame("/PNP_enfant").attachTo(
					legLift.getFrame("/Dummy/PNP_parent"));
			// robot.setSafetyWorkpiece(leg1k5); //déclare en sécurité
			while (answer != 1) {
				for (int i = 0; i < nbcycle; i++) {
					leg1k5.getFrame("Genoux").move(
							linRel(0, 0, 0, Math.toRadians(-angle), 0, 0)
									.setCartVelocity(vitesse));
					leg1k5.getFrame("Genoux").move(
							linRel(0, 0, 0, Math.toRadians(angle), 0, 0)
									.setCartVelocity(vitesse));
				}
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"Voulez vous refaire un cycle ?", "Oui", "Non");
			}
			answer = -1;
			ThreadUtil.milliSleep(tempo);// 10 sec pour détacher sa jambe
			while (answer != 0) {
				ThreadUtil.milliSleep(5000);// détache la jambe
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"La jambe de " + nom + " est elle enlevée ?", "Oui",
						"Non");
			}
			answer = -1;
			leg1k5.detach();// detache la jambe de l'outil en logiciel
			// robot.setSafetyWorkpiece(null);
			robot.move(ptpHome().setJointVelocityRel(0.5));
			break;

		case mathis:
			// your application execution starts here
			robot.move(ptpHome().setJointVelocityRel(0.5));
			legLift.getFrame("/Dummy/PNP_parent").move(
					ptp(getApplicationData().getFrame("/Genoux/P1")));
			// message variable
			// answer=getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
			// "La jambe du patient est elle en place ?", "Oui","Non");
			while (answer != 0) {
				ThreadUtil.milliSleep(5000);// attache la jambe
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"La jambe de " + nom + " est elle en place ?", "Oui",
						"Non");
			}
			answer = -1;
			leg1k5.getFrame("/PNP_enfant").attachTo(
					legLift.getFrame("/Dummy/PNP_parent"));
			// robot.setSafetyWorkpiece(leg1k5); //déclare en sécurité
			while (answer != 1) {
				leg1k5.getFrame("Genoux").move(
						ptp(getApplicationData().getFrame(
								"/Genoux/point_centre_droite")));
				leg1k5.getFrame("Genoux")
						.move(ptp(getApplicationData().getFrame(
								"/Genoux/point_haut")));
				leg1k5.getFrame("Genoux").move(
						ptp(getApplicationData().getFrame(
								"/Genoux/point_centre_gauche")));
				leg1k5.getFrame("Genoux").move(
						ptp(getApplicationData().getFrame("/Genoux/P1")));
				for (int j = 0; j < nbcycle; j++) {
					leg1k5.getFrame("Genoux").move(
							linRel(0, 0, 0, Math.toRadians(-angle), 0, 0)
									.setCartVelocity(vitesse));
					leg1k5.getFrame("Genoux").move(
							linRel(0, 0, 0, Math.toRadians(angle), 0, 0)
									.setCartVelocity(vitesse));
				}
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"Voulez vous refaire un cycle ?", "Oui", "Non");
			}
			answer = -1;
			ThreadUtil.milliSleep(tempo);// 10 sec pour détacher sa jambe
			while (answer != 0) {
				ThreadUtil.milliSleep(5000);// détache la jambe
				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.QUESTION,
						"La jambe de " + nom + " est elle enlevée ?", "Oui",
						"Non");
			}
			answer = -1;
			leg1k5.detach();// detache la jambe de l'outil en logiciel
			// robot.setSafetyWorkpiece(null);
			robot.move(ptpHome().setJointVelocityRel(0.5));
			break;

		default:
			getApplicationUI()
					.displayModalDialog(ApplicationDialogType.ERROR,
							"Le nom séléctionné n'existe pas, pensez à écrire le nom en majuscule");
			break;
		}
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