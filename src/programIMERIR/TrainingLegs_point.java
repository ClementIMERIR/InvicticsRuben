package programIMERIR;


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
public class TrainingLegs_point extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	@Inject
	@Named("LegLift")
	private Tool legLift;//Création d'un objet outil
	@Inject
	@Named("Leg_thomas")
	private Tool leg;
	@Inject
	@Named("Leg1k5")
	private Workpiece leg1k5;
	
	//variable accessible via process data
	private Integer tempo, 
					nbcycle;
	private Double angle;
	private Double vitesse;
	private String name;
	
	//---------------------
	private int answer;
	private String nom;
	
	private String key = "1,5";
	
	private enum Personne {
	    PIERRE, PAUL, JACK;
	}
	
	double centre_gravity (double diametre, double longueur){
		double x = (diametre/2);
		double y = (longueur/2);
		double z = (diametre/2);
		return 0;	
	}
	
	int moment_inertie (double diametre, double longueur, double masse){
		double D2 = Math.pow(2, diametre);
		double L2 = Math.pow(2, longueur);
		double Jx = masse*((L2/12)+(D2/16));
		double Jy = ((masse*D2)/8);
		double Jz = Jx;
		return 0;
	}
	
	
	
	@Override
	public void initialize() {
		legLift.attachTo(robot.getFlange());//"Fixation" de l'outil à la bride du robot.
		nom = getApplicationData().getProcessData("name").getValue();
		answer = -1; //initialize a une valeur nom utilisable par la boite de dialogue
		
		
	} 
	
		/*
		name Nom = 
		switch (name)
		{
		  case name.:
			  //tempo = getApplicationData().getProcessData("tempo").setValue(10000);
			    getApplicationData().getProcessData("tempo").setValue(10000);
		    break;        
		  default:
		    /*Action/;             
		}
		
		/*
		 <processData displayName="Temporisation" dataType="java.lang.Integer" defaultValue="10000" id="tempo" min="100" max="100000" value="10000" unit="milliseconde" comment="Temporisation pour attendre la jambe du patient"/>
      <processData displayName="Nombre_cycle" dataType="java.lang.Integer" defaultValue="5" id="nbcycle" min="2" max="10" value="5" unit="cycle" comment="Nombre de cycle"/>
  	  <processData displayName="Debatement_angulaire" dataType="java.lang.Double" defaultValue="20" id="angle" min="5" max="40" value="30" unit="°" comment="Angle de deplacement de la jambe pendant la scéance"/>
  	  <processData displayName="Vitesse_angulaire" dataType="java.lang.Double" defaultValue="20" id="vitesse" min="5" max="40" value="30" unit="mm/s" comment="Vitesse de deplacement de la jambe pendant la scéance"/>
		 
		 */
		
		/*
		// initialize your application here
		legLift.attachTo(robot.getFlange());//"Fixation" de l'outil à la bride du robot.	
		tempo = getApplicationData().getProcessData("tempo").getValue();
		nbcycle = getApplicationData().getProcessData("nbcycle").getValue();
		angle = getApplicationData().getProcessData("angle").getValue();
		vitesse = getApplicationData().getProcessData("vitesse").getValue();
		}*/

	@Override
	public void run() {
		//choix nom
		tempo = getApplicationData().getProcessData("tempo").getValue();
		nbcycle = getApplicationData().getProcessData("nbcycle").getValue();
		angle = getApplicationData().getProcessData("angle").getValue();
		vitesse = getApplicationData().getProcessData("vitesse").getValue();

	    	// your application execution starts here
			robot.move(ptpHome().setJointVelocityRel(0.5));
			legLift.getFrame("/Dummy/PNP_parent").move(ptp(getApplicationData().getFrame("/Genoux/P1")));
			//message variable answeranswer=getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "La jambe du patient est elle en place ?", "Oui","Non");
	
			leg.getFrame("/PNP_enfant").attachTo(legLift.getFrame("/Dummy/PNP_parent"));
			while(answer !=1){
				for(int i=0;i<5;i++){
					leg.getFrame("Genoux").move(linRel(0,0,0,Math.toRadians(-angle),0,0).setCartVelocity(vitesse));
					leg.getFrame("Genoux").move(linRel(0,0,0,Math.toRadians(angle),0,0).setCartVelocity(vitesse));
				}
			}
	}
}
