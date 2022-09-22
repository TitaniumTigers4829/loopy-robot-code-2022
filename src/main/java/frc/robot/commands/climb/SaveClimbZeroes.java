package frc.robot.commands.climb;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class SaveClimbZeroes {
  
    /**
     * This class is used to save the climb hook values as a text file on the rio.
     * The first line of the text file is for the left hook height, second for the right.
     */
    public SaveClimbZeroes() {}

    public static void makeSureFileExists() {
        try {
    		File file = new File(ClimbConstants.kClimbTextFilePath);
    		if(!file.exists()){
                file.mkdirs();
    			file.createNewFile();
    		}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
            SmartDashboard.putString("Error 3", e.getMessage());
		}
    }

    public static void writeClimbZeroes(double leftClimbHookZero, double rightClimbHookZero) {
 
        try {
            BufferedWriter fileWriter = new BufferedWriter(
                new FileWriter(ClimbConstants.kClimbTextFilePath)
            );
 
            fileWriter.write(Double.toString(leftClimbHookZero));
            fileWriter.write(Double.toString(rightClimbHookZero));

            fileWriter.close();
        }
        // Catch block to handle if exceptions occurs
        catch (IOException e) {
            // Print the exception on console, use smart dashboard if debugging is needed
            SmartDashboard.putString("Error 1", e.getMessage());
            System.out.print(e.getMessage());
        }
    }

    public static double[] readClimbZeroes() {
         
        try {
            BufferedReader fileReader = new BufferedReader(
                new FileReader(ClimbConstants.kClimbTextFilePath)
            );
    
            String leftClimbHookZero = fileReader.readLine();
            String rightClimbHookZero = fileReader.readLine();

            fileReader.close();

            double[] climbZeroes = {Double.parseDouble(leftClimbHookZero), Double.parseDouble(rightClimbHookZero)};

            return climbZeroes;
        }
        // Catch block to handle if exceptions occurs
        catch (IOException e) {
            // Print the exception on console, use smart dashboard if debugging is needed
            System.out.print(e.getMessage());
            SmartDashboard.putString("Error 2", e.getMessage());
        }
        // TODO: Get average climb zeroes
        return new double[] {0, 0};
    }

}
